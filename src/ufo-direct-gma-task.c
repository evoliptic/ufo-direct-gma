/*
 * Copyright (C) 2011-2015 Karlsruhe Institute of Technology
 *
 * This file is part of Ufo.
 *
 * This library is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */


/* ToDO:
   -cf dma perf_counter and perf with lorenzo
   - auto-detect of streaming and board gen : lorenzo should update the firmware, so we keep at least the board gen thing.
   -cf verifying the streaming -> if streaming is not activated on the firmware of the board, then this task should not work
   -if NDA is signed with AMD, then it could be good to see on how getting the aperture automatically without being root
   -review
*/

#define _POSIX_C_SOURCE 200809L
#define _BSD_SOURCE
#define _XOPEN_SOURCE 700
#include "ufo-direct-gma-task.h"

#ifdef __APPLE__
/* i'm not sure about directgma working on mac os, so let's put nothing for now*/
#else
#include "CL/cl.h"
#include "CL/cl_ext.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <sched.h>
#include <sys/time.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sched.h>
#include <errno.h>

#include <pcilib.h>
#include <pcilib/kmem.h>
#include <pcilib/bar.h>
#include <pcilib/error.h>
#include <sys/mman.h>
#include <fcntl.h>

#define DEVICE "/dev/fpga0"

#define IPECAMERA
#define BAR PCILIB_BAR0
#define USE_RING PCILIB_KMEM_USE(PCILIB_KMEM_USE_USER, 1)
#define USE PCILIB_KMEM_USE(PCILIB_KMEM_USE_USER, 2)
#define DESC_THRESHOLD  1

/**
 * defines for aperture size of GPU
 */
#define SIZE_128_M      128000000
#define SIZE_96_M       96000000

#define PAGE_SIZE       4096        // other values are not supported in the kernel

#define FPGA_CLOCK      250

/**
 * macro to write to a given adress in the FPGA
 */
#define WR(addr, value) { *(uint32_t*)((bar) + (addr) + (offset)) = value; }

/**
 * macro to read a register from a register
 */
#define RD(addr, value) { value = *(uint32_t*)((bar) + (addr) + (offset)); }

/**
 * list of registers used in the FPGA
 */
#define RESET_DMA 0x00
#define DMA_START 0x04
#define PACKET_PARAM 0x0C
#define NB_PACKETS_BY_DESC 0x10
#define PERF_COUNTER 0x28
#define MAX_PAYLOAD_SIZE 0x40
#define DESCRIPTOR_MEMORY 0x50
#define UPDATE_REG 0x54
#define LAST_REG_READ_DRIVER 0x58
#define NB_DESCRIPTORS_FPGA 0x5C
#define THRESHOLD_REG 0x60
#define ENABLE_COUNTER 0x9000
#define CONTROL_FRAME_GEN 0x9040
#define START_LINE 0x9160
#define SKIP_LINE 0x9164
#define NUMBER_ROWS 0x9168
#define NUMBER_FRAMES 0x9170
#define ADD_NUMBER_OF_TRIGGERS 0x9180

#define NB_ROWS 3840

struct _UfoDirectGmaTaskPrivate {
    guint huge_page;
    guint tlp_size;
    guint multiple;
    guint buffers;
    cl_context context;
    cl_platform_id platform_id;
    guint width;
    guint height;
    guint frames;
    guint64 start_index;
    guint64 stop_index;
    glong* buffer_gma_addr;
    UfoBuffer **buffers_gma;
    cl_command_queue command_queue;
    guintptr *bus_addr;
    volatile gpointer bar;
    pcilib_t *pci;
    guintptr kdesc_bus;
    volatile guint32 *desc;
    guint board_gen;
    guint print_perf;
    guint print_index;
    guint print_counter;
    guint counter;
    guint get_ap_size;
    guint mode;
    guint iterations;
    guint error;
    guint nb_frames;
};

static void ufo_task_interface_init (UfoTaskIface *iface);

G_DEFINE_TYPE_WITH_CODE (UfoDirectGmaTask, ufo_direct_gma_task, UFO_TYPE_TASK_NODE,
                         G_IMPLEMENT_INTERFACE (UFO_TYPE_TASK,
                                                ufo_task_interface_init))

#define UFO_DIRECT_GMA_TASK_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE((obj), UFO_TYPE_DIRECT_GMA_TASK, UfoDirectGmaTaskPrivate))

enum {
    PROP_0,
    PROP_HUGE_PAGE,
    PROP_TLP_SIZE,
    PROP_MULTIPLE,
    PROP_BUFFERS,
    PROP_WIDTH,
    PROP_HEIGHT,
    PROP_FRAMES,
    PROP_COUNTER,
    PROP_START_INDEX,
    PROP_STOP_INDEX,
    PROP_PRINT_PERF,
    PROP_PRINT_COUNTER,
    PROP_PRINT_INDEX,
    PROP_GET_AP_SIZE,
    PROP_ITERATIONS,
    PROP_NB_FRAMES,
    N_PROPERTIES
};

static GParamSpec *properties[N_PROPERTIES] = { NULL, };

UfoNode *
ufo_direct_gma_task_new (void)
{
    return UFO_NODE (g_object_new (UFO_TYPE_DIRECT_GMA_TASK, NULL));
}

static void
ufo_direct_gma_task_get_requisition (UfoTask *task,
                                     UfoBuffer **inputs,
                                     UfoRequisition *requisition)
{
    UfoDirectGmaTaskPrivate *priv;
    priv = UFO_DIRECT_GMA_TASK_GET_PRIVATE (task);

    requisition->n_dims = 2;
    requisition->dims[0] = priv->width;
    requisition->dims[1] = priv->height;
}

static guint
ufo_direct_gma_task_get_num_inputs (UfoTask *task)
{
    return 0;
}

static guint
ufo_direct_gma_task_get_num_dimensions (UfoTask *task,
                                        guint input)
{
    return 0;
}

static UfoTaskMode
ufo_direct_gma_task_get_mode (UfoTask *task)
{
  return UFO_TASK_MODE_GENERATOR | UFO_TASK_MODE_GPU;
}

/**
 * function to initialize a cl_meme buffer
 */
static void
init_buffer_gma (UfoBuffer** buffer, cl_command_queue* command_queue, int init)
{
    ufo_buffer_init_gma (*buffer, &init, command_queue);
}

/**
 * function to print the values of the given buffer between start and stop
 */
static void
printf_with_index (guint start, guint stop, int* buffer)
{
    guint i;
    printf("index print : %i %i\n",start,stop);

    for (i = start; i < stop; i++)
        printf("%x |", buffer[i]);

    printf("\n");
}

/**
 * this function get the aperture size of the GPU in order to define the
 * mode(direct or multiple buffering) of the transfer later
 */
static int
verify_aperture_size (UfoDirectGmaTaskPrivate *priv)
{
    guint aperture_size;

    if (priv->get_ap_size==1){
        /* here, we get the aperture size automatically : the current version is not good and requires root. cf lorenzo afterwards*/
        FILE *fp = popen ("aticonfig --get-pcs-key=MCIL,DMAOGLExtensionApertureMB | grep -o '[0-9][0-9])$' | grep -o '[0-9][0-9]' ", "r");
        fscanf (fp, "%u", &aperture_size);
        aperture_size *= 1000000;
    }
    else if (priv->get_ap_size == 0){
        aperture_size = SIZE_96_M;
    }
    else if (priv->get_ap_size == 2){
        aperture_size = SIZE_128_M;
    }

#ifdef DEBUG
    printf ("aperture size obtained: %u \n", aperture_size);
#endif

    if ((priv->buffers * priv->huge_page * 4096) > aperture_size) {
        pcilib_error ("the size for buffers for gma is higher than the aperture size\n");
        return 1;
    }

    if ((priv->buffers * priv->multiple * priv->huge_page) > 1048576) {
        pcilib_error ("the total size is too big\n");
        return 1;
    }

    /* in the case of frames, a frame is 37MB but with decode and such becomes 75MB, so the two factor*/
    if ((priv->frames==1) && ((priv->width * priv->height * priv->nb_frames * 2) > aperture_size))
        priv->mode = 1;
    /*in the case of counter, just do it normally*/
    else if ((priv->counter==1) && ((priv->width * priv->height * priv->nb_frames) > aperture_size))
        priv->mode=1;
    else
        priv->mode = 0;

#ifdef DEBUG
    printf("mode obtained : %i\n", priv->mode);
#endif
    return 0;
}

/**
 * this function create a buffer for directgma in the multiple buffering mode
 */
static glong
create_gma_buffer (UfoBuffer** buffer, UfoDirectGmaTaskPrivate *priv, cl_bus_address_amd* busadress, cl_command_queue *command_queue)
{
    *buffer = (UfoBuffer*) ufo_buffer_new_with_size_in_bytes (1024 * priv->huge_page * sizeof(int), priv->context);
    /* ufo_buffer_set_location(*buffer,UFO_BUFFER_LOCATION_DEVICE_DIRECT_GMA); */
    ufo_buffer_get_device_array_for_directgma (*buffer,command_queue,priv->platform_id,busadress);

    return busadress->surface_bus_address;
}

/**
 * this function gets the board generation, in order to have correct transfer parameters afterwards
 */
static void
get_board_generation(UfoDirectGmaTaskPrivate *priv)
{
  gint value;
  volatile void* bar=priv->bar;
  guintptr offset=0;

  int compare=7;
  
  RD(0x18,value);

  if(value!=18 && value!=19){
    pcilib_warning("board generation can't be found automatically, switching to default gen3");
    priv->board_gen=3;
  }

  int g=value&compare;
  priv->board_gen=g;

}

/**
 * this function gets if the streaming is activated for the board, we should stop the program if not.
 */
static gboolean
verify_streaming(UfoDirectGmaTaskPrivate *priv)
{
  
  volatile void* bar=priv->bar;
  guintptr offset=0;
  gint value;
  int constant=1<<4;
  RD(0x18,value);
  if(value&constant) return TRUE;
  else return FALSE;
}

/**
 * gpu initialization in multiple buffering mode
 */
static gboolean
gpu_init (UfoTask* task)
{
    cl_bus_address_amd* busadresses;
    guint i;
    UfoGpuNode *node;
    UfoDirectGmaTaskPrivate *priv;
    priv= UFO_DIRECT_GMA_TASK_GET_PRIVATE(task);


    /*get the gpu and the command queue*/
    node = UFO_GPU_NODE (ufo_task_node_get_proc_node (UFO_TASK_NODE (task)));
    priv->command_queue = ufo_gpu_node_get_cmd_queue (node);

    busadresses = malloc(priv->buffers*sizeof(cl_bus_address_amd));

    int* results;
    results = malloc(priv->huge_page*priv->buffers*1024*sizeof(int));

    /* we create here a list of buffers, said directgma buffers, where the transfer will be done, and from where data will be copied to a final buffer*/
    for (i = 0; i < priv->buffers; i++) {
        priv->buffer_gma_addr[i] = create_gma_buffer (&(priv->buffers_gma[i]), priv, &busadresses[i], &(priv->command_queue));

#ifndef PERF_MAX
        init_buffer_gma (&(priv->buffers_gma[i]), &(priv->command_queue),42);
#endif

        ufo_buffer_read (priv->buffers_gma[i], results, &(priv->command_queue)); /**< this line has an impact on the data integrity, why, i don't know*/

#ifdef DEBUG
        printf("\n buffer directgma %i\n",i);
        if (priv->print_index == 1)
            printf_with_index (priv->start_index,priv->stop_index,results);
#endif
        if (priv->buffer_gma_addr[i] == 0) {
            pcilib_error ("the buffer %i for directgma has not been allocated correctly\n");
            return FALSE;
        }
    }
#ifdef DEBUG
    free(results);
#endif

    return TRUE;
}

/**
 * gpu iniitalization in the direct mode
 */
static gboolean
gpu_init_mode0 (UfoTask* task)
{
    UfoGpuNode *node;
    UfoDirectGmaTaskPrivate *priv;

    /*get the gpu and the command queue*/
    priv = UFO_DIRECT_GMA_TASK_GET_PRIVATE (task);
    node = UFO_GPU_NODE (ufo_task_node_get_proc_node (UFO_TASK_NODE (task)));
    priv->command_queue = ufo_gpu_node_get_cmd_queue (node);

    return TRUE;
}

/**
 * output buffer initialization in multiple buffering mode
 */
static void
gpu_init_for_output (UfoBuffer **saving_buffers, UfoDirectGmaTaskPrivate* priv)
{
  /* in this mode, we allocate an output buffer as a traditionnal opencl buffer, which size is equal to the size requested*/
    /* ufo_buffer_set_location(*saving_buffers, UFO_BUFFER_LOCATION_DEVICE); */
    ufo_buffer_get_device_array (*saving_buffers, &(priv->command_queue));

#ifndef PERF_MAX
    init_buffer_gma (saving_buffers,&(priv->command_queue), 666);
#endif

#ifdef DEBUG
    int* results;
    results = malloc(priv->width * priv->height * priv->nb_frames * sizeof(int));
    printf("final buffer\n");
    ufo_buffer_read (*saving_buffers, results, &(priv->command_queue));

    if (priv->print_index == 1)
        printf_with_index(priv->start_index, priv->stop_index, results);

    free(results);
#endif
}

/**
 * output buffer initialization in direct mode
 */
static void
gpu_init_for_output_mode0 (UfoBuffer **saving_buffers, UfoDirectGmaTaskPrivate* priv)
{
    guint j;
    cl_bus_address_amd busaddress;
    /* in this mode, the output buffer is used directly for directgma, we allocate it for that so*/
    /* ufo_buffer_set_location(*saving_buffers, UFO_BUFFER_LOCATION_DEVICE_DIRECT_GMA); */
    ufo_buffer_get_device_array_for_directgma (*saving_buffers, &(priv->command_queue), priv->platform_id, &busaddress);

#ifndef PERF_MAX
    init_buffer_gma (saving_buffers, &(priv->command_queue), 69);
#endif

#ifdef DEBUG
    int* results;
    results = malloc (priv->multiple * priv->huge_page * priv->buffers * 1024 * sizeof(int));
    printf("final buffer\n");
    ufo_buffer_read (*saving_buffers, results, &(priv->command_queue));

    if (priv->print_index == 1)
        printf_with_index (priv->start_index, priv->stop_index, results);

    free(results);
#endif

    /* after the output buffer has been allocated, we get sub-buffers(buffers
     * that are part of the output buffer) adresses from the output buffer, and
     * register them for address table for the fpga*/
    priv->buffer_gma_addr[0] = busaddress.surface_bus_address;
#ifdef DEBUG
    printf("gma buffer 0 addr: %lu  \n", priv->buffer_gma_addr[0]);
#endif

    for (j = 1; j < priv->buffers; j++) {
        priv->buffer_gma_addr[j] = priv->buffer_gma_addr[j-1] + 4096 * priv->huge_page;
#ifdef DEBUG
        printf("gma buffer %u addr: %lu\n", j, priv->buffer_gma_addr[j]);
#endif
    }
}

/**
 * function ot verify the board is ready for PCIe bus mastering
 */
static gboolean
pcie_test (volatile gpointer bar)
{
    guintptr offset = 0;
    gint err;
#ifdef DEBUG
    printf("DMA reset....\n");
#endif
    WR(RESET_DMA, 0x1);
    usleep(100000);
    WR(RESET_DMA, 0x0);
    usleep(100000);

    RD(RESET_DMA, err);

    if (err == 335746816 || err == 335681280) {
#ifdef DEBUG
        printf ("\xE2\x9C\x93 \n");
#endif
    }
    else {
        printf ("PCIe not ready!\n");
        return FALSE;
    }

    return TRUE;
}

/**
 * function to set up several parameters for dma
 */
static void
dma_conf (UfoDirectGmaTaskPrivate* priv)
{
    guintptr offset = 0;
    volatile gpointer bar = priv->bar;

#ifdef DEBUG
    printf ("DMA: send data mount\n");
#endif
    WR (NB_PACKETS_BY_DESC, (priv->huge_page * (PAGE_SIZE / (4 * priv->tlp_size))));
#ifdef DEBUG
    printf ("DMA: putting running mode\n");
#endif

    if (priv->board_gen == 3) {
        WR(PACKET_PARAM, 0x80000 | priv->tlp_size);
    }
    else {
        WR(PACKET_PARAM, priv->tlp_size);
    }

    WR(NB_DESCRIPTORS_FPGA, 0x00);
}

/**
 * driver initialization for directgma transfer
 */
static void
pcilib_init_for_transfer (UfoDirectGmaTaskPrivate* priv)
{
    pcilib_kmem_handle_t *kdesc;
    pcilib_kmem_flags_t flags = PCILIB_KMEM_FLAG_HARDWARE | PCILIB_KMEM_FLAG_PERSISTENT | PCILIB_KMEM_FLAG_EXCLUSIVE;
    pcilib_kmem_flags_t clean_flags = PCILIB_KMEM_FLAG_HARDWARE | PCILIB_KMEM_FLAG_PERSISTENT | PCILIB_KMEM_FLAG_EXCLUSIVE;
    pcilib_bar_t bar_tmp = BAR;
    uintptr_t offset = 0;

    (priv->pci) = pcilib_open (DEVICE, "pci");

    if (!(priv->pci))
        pcilib_error ("pcilib_open");

    priv->bar = pcilib_map_bar (priv->pci, BAR);

    if (!(priv->bar)) {
        pcilib_close (priv->pci);
        pcilib_error ("map bar");
    }

    pcilib_detect_address (priv->pci, &bar_tmp, &offset, 1);

    pcilib_enable_irq (priv->pci, PCILIB_IRQ_TYPE_ALL, 0);
    pcilib_clear_irq (priv->pci, PCILIB_IRQ_SOURCE_DEFAULT);

    pcilib_clean_kernel_memory (priv->pci, USE, clean_flags);
    pcilib_clean_kernel_memory (priv->pci, USE_RING, clean_flags);

    kdesc = pcilib_alloc_kernel_memory (priv->pci, PCILIB_KMEM_TYPE_CONSISTENT, 1, 128, 4096, USE_RING, flags);
    priv->kdesc_bus = pcilib_kmem_get_block_ba (priv->pci, kdesc, 0);
    priv->desc = (uint32_t*) pcilib_kmem_get_block_ua (priv->pci, kdesc, 0);
    memset((void*) priv->desc, 0, 5*sizeof(uint32_t));
#ifdef DEBUG
    printf ("bar debut: %p\n",priv->bar);
#endif
}

/**
 * function to write the adress table of the fpga
 */
static void
writing_dma_descriptors (UfoDirectGmaTaskPrivate* priv)
{
    uintptr_t offset = 0;
    guint j;
    volatile void* bar=priv->bar;

#ifdef DEBUG
    printf ("Writing SW Read Descriptor\n");
    printf ("bar adress: %p\n", bar);
    printf ("nb buffers: %i\n", priv->buffers);
#endif

    WR (LAST_REG_READ_DRIVER, priv->buffers-1);

#ifdef DEBUG
    printf ("Writing the Descriptor Threshold\n");
#endif

    WR(THRESHOLD_REG, DESC_THRESHOLD);
    WR(UPDATE_REG, priv->kdesc_bus);
    usleep(100000);

    for (j = 0; j < priv->buffers; j++) {
        priv->bus_addr[j] = priv->buffer_gma_addr[j];
        usleep(1000);
#ifdef DEBUG
        printf("Writing descriptor num. %i: \t %08lx \n", j, priv->bus_addr[j]);
#endif
        WR(DESCRIPTOR_MEMORY, priv->bus_addr[j]);
    }
}

/**
 * the transfer itself in multiple buffering mode
 */
static guint
handshaking_dma (UfoBuffer* saving_buffers, UfoDirectGmaTaskPrivate* priv)
{
    guint i;
    uintptr_t offset = 0;
    guint32 curptr, hwptr, curbuf;
    gint err;
    volatile void* bar = priv->bar;

#ifdef DEBUG2
    int* results;
    results = malloc (priv->huge_page * priv->buffers * 1024 * sizeof(int));
#endif
    i = 0;
    curptr = 0;
    curbuf = 0;
    GTimer *timer2 = g_timer_new ();

    while (i < priv->multiple) {
        /* get the state of dma*/
        do {
            hwptr = priv->board_gen == 3 ? priv->desc[3] : priv->desc[4];
        }
        while (hwptr == curptr);

        /* retrieve driver latency*/
        do {
            /* copy from a directgma buffer to the output buffer, and wait for the copy to be finished, to make sure to not write again in a buffer not copied*/
            err = ufo_buffer_copy_for_directgma (priv->buffers_gma[curbuf], saving_buffers, (i * priv->buffers + curbuf), &(priv->command_queue));
#ifdef DEBUG2
            printf ("loop %i with curbuf %i\n", i, curbuf);
            ufo_buffer_read (priv->buffers_gma[curbuf], results, &(priv->command_queue));
            if (priv->print_index == 1)
                printf_with_index (priv->start_index, priv->stop_index, results);
#endif
            if (err == CL_INVALID_VALUE)
                break;

	    if (i < (priv->multiple-1) || (i == (priv->multiple-1) && curbuf < 1)){
                    if (priv->desc[1] == 0)
                        WR(DESCRIPTOR_MEMORY, priv->bus_addr[curbuf]);
            }

            curbuf++;
            if (curbuf == priv->buffers) {
                i++;
                curbuf = 0;
                if (i >= priv->multiple)
                    break;
            }
        }
        while (priv->bus_addr[curbuf] != hwptr);

        if (err==CL_INVALID_VALUE)
            break;

        if (priv->board_gen == 3) {
            if (priv->desc[1] != 0) {
                err = ufo_buffer_copy_for_directgma (priv->buffers_gma[curbuf], saving_buffers, (i*priv->buffers+curbuf), &(priv->command_queue));
                break;
            }
        }
        else {
            if (priv->desc[2] != 0){
                if (priv->bus_addr[curbuf] == hwptr) {
                    err = ufo_buffer_copy_for_directgma (priv->buffers_gma[curbuf], saving_buffers, (i*priv->buffers+curbuf), &(priv->command_queue));
                    break;
                }
            }
        }
        curptr = hwptr;
    }

    g_timer_stop (timer2);

    if (priv->print_perf == 1)
        g_print ("transfer: %fs\n", g_timer_elapsed (timer2, NULL));

    g_timer_destroy (timer2);

#ifdef DEBUG2
    free(results);
#endif

    return curbuf != 0 ? i * priv->buffers + curbuf : i * priv->buffers + curbuf - 1;
}

/**
 * the transfer itself in the direct mode
 */
static guint
handshaking_dma_mode0 (UfoBuffer* saving_buffers, UfoDirectGmaTaskPrivate* priv)
{
    guint i;
    uintptr_t offset = 0;
    guint32 curptr, hwptr, curbuf;
    volatile void* bar = priv->bar;

    i = 0;
    curptr = 0;
    curbuf = 0;
    GTimer *timer = g_timer_new ();

    /* same as above, without any copy. the number 2 for i is to make sure the transfer is finished*/
    while (i < 2) {
        do {
            hwptr = priv->board_gen == 3 ? priv->desc[3] : priv->desc[4];
        }
        while (hwptr == curptr);

        do {
            if ((curbuf < 1) && (priv->desc[1] == 0))
                WR (DESCRIPTOR_MEMORY, priv->bus_addr[curbuf]);

            curbuf++;

            if (curbuf == priv->buffers) {
                i++;
                curbuf = 0;
                if (i >= priv->multiple)
                    break;
            }
        }
        while (priv->bus_addr[curbuf] != hwptr);

        if ((priv->board_gen == 3) && (priv->desc[1] != 0))
            break;
        else if ((priv->desc[2] != 0) && (priv->bus_addr[curbuf] == hwptr))
            break;

        curptr = hwptr;
    }

    g_timer_stop (timer);

    if (priv->print_perf == 1)
        g_print ("transfer: %fs\n", g_timer_elapsed (timer, NULL));

    g_timer_destroy (timer);

    return curbuf != 0 ? curbuf : curbuf - 1;
}

/**
 * function to stop the dma
 */
static void
stop_dma (struct timeval *end, gfloat* perf_counter, volatile void* bar)
{
    uintptr_t offset = 0;
#ifdef DEBUG
    printf("dma stop\n");
#endif
    gettimeofday (end, NULL);
    WR (DMA_START, 0x00);
    usleep (100);
    RD (PERF_COUNTER, *perf_counter); /**< to see*/
    usleep (100);
    WR (RESET_DMA, 0x01);
}

/**
 * function to free the memory allocations and close driver in multiple buffering mode
 */
static void
free_and_close (UfoDirectGmaTaskPrivate* priv)
{
    guint j;
    free(priv->bus_addr);

    for (j = 0; j < priv->buffers; j++)
        g_object_unref (priv->buffers_gma[j]);

    free (priv->buffer_gma_addr);
    pcilib_disable_irq (priv->pci, 0);
    pcilib_unmap_bar (priv->pci, BAR, priv->bar);
    pcilib_close (priv->pci);
}

/**
 * function to free the memory allocations and close driver in direct mode
 */
static void
free_and_close_mode0 (UfoDirectGmaTaskPrivate* priv)
{
    free (priv->bus_addr);
    free (priv->buffer_gma_addr);
    pcilib_disable_irq (priv->pci, 0);
    pcilib_unmap_bar (priv->pci, BAR,priv->bar);
    pcilib_close (priv->pci);
}

/**
 * function to start the dma engine and the data generator, given the parameters
 */
static void
start_dma(UfoDirectGmaTaskPrivate* priv){
    guintptr offset = 0;
    gint nb_frames;
    gint nb_rows;
    volatile void* bar = priv->bar;

#ifdef DEBUG
    printf ("putting data generator\n");
    printf ("frames %i counter %i\n",priv->frames, priv->counter);
#endif

    if (priv->frames == 1){
        /* here we get frames data generator option*/
        nb_frames = priv->height / NB_ROWS;
        nb_rows = NB_ROWS;
        WR(ENABLE_COUNTER, 0x0);
        usleep(100);

        WR(CONTROL_FRAME_GEN,0xf);
        usleep(100);

        WR(0x9100,0);
        usleep(100);

        WR(START_LINE,0x0);
        usleep(100);

        WR(SKIP_LINE,0x0);
        usleep(100);

        WR(NUMBER_ROWS,nb_rows);
        usleep(100);

        WR(NUMBER_FRAMES,nb_frames);
        usleep(100);

        WR(ADD_NUMBER_OF_TRIGGERS,0);
        usleep(100);

        WR(CONTROL_FRAME_GEN,0xfff000);
        usleep(100);
    }
    else if (priv->counter == 1){
        /* here we get counter data generator option*/
        WR(NUMBER_ROWS,0);
        usleep(100);

        WR(NUMBER_FRAMES,0);
        usleep(100);

        WR(CONTROL_FRAME_GEN, 0x0);
        usleep(100);

        WR(ENABLE_COUNTER, 0xff);
        usleep(100);

        WR(ENABLE_COUNTER, 0x1);
    }
    WR(DMA_START, 0x1);
}

/**
 * function to print the performance of the transfer (perf_counter on fpga not properly accurate)
 */
static void
perf (struct timeval start, struct timeval end, float perf_counter, UfoDirectGmaTaskPrivate* priv, guint buffers_completed)
{
    gfloat performance;
    gsize run_time;
    gfloat size_mb;

    run_time = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
    size_mb = buffers_completed * priv->huge_page / 256;
    printf ("Performance: transfered %f Mbytes in %zu us using %d buffers\n", (size_mb), run_time, priv->buffers);
    performance = ((size_mb * FPGA_CLOCK * 1000000) / (perf_counter * 256));
    printf ("DMA perf counter:\t%d\n", (int) perf_counter);
    printf ("DMA side:\t\t%.3lf MB/s\n", performance);
    printf ("PC side:\t\t%.3lf MB/s\n\n", 1000000. * size_mb / run_time );
}

/**
 * this function verify that there is no jump in the counter
 */
static void
research_data_fail_counter (int* buffer, UfoDirectGmaTaskPrivate* priv)
{
    guint i;
    guint k = 0;
    for (i = 1; i < priv->multiple * priv->huge_page * priv->buffers * 1024-1; i++) {
        if (buffer[i+1] - buffer[i] != 1) {
            printf("problem at position %i: %i %i %i %i\n", i, buffer[i-1],buffer[i], buffer[i+1],buffer[i+2]);
            k++;
        }
    }

    if (k == 0)
        printf("no problem in data\n");
}

/**
 * function to select what we want to print as result
 */
static void
print_results (UfoBuffer* buffer, UfoDirectGmaTaskPrivate* priv)
{
    int* results;
    results = malloc (priv->multiple * priv->huge_page * priv->buffers * 1024 * sizeof(int));
    printf("results: \n");
    ufo_buffer_read (buffer, results, &(priv->command_queue));

    if (priv->print_counter == 1)
        research_data_fail_counter (results, priv);

    if (priv->print_index == 1)
        printf_with_index (priv->start_index, priv->stop_index, results);

    free (results);
}

static void
ufo_direct_gma_task_setup (UfoTask *task,
			               UfoResources *resources,
			               GError **error)
{
    UfoDirectGmaTaskPrivate *priv;
    priv = UFO_DIRECT_GMA_TASK_GET_PRIVATE (task);
    gint err;

#ifdef DEBUG
    printf ("setup start\n");
#endif
    priv->context = ufo_resources_get_context (resources);
    ufo_get_platform_id_for_directgma (resources, &(priv->platform_id));

    if (priv->frames == 1) {
        priv->height *= priv->nb_frames;
        priv->multiple *= priv->nb_frames;
    }

    if ((err = verify_aperture_size (priv))==1){
        g_set_error (error, UFO_TASK_ERROR, UFO_TASK_ERROR_SETUP,
                     "Could not verify aperture size");
        priv->error = 1;
        return;
    }

    priv->buffer_gma_addr = malloc (priv->buffers * sizeof(glong));
    priv->buffers_gma = malloc (priv->buffers * sizeof(UfoBuffer*));
    priv->bus_addr = malloc (priv->buffers * sizeof(uintptr_t));

#ifdef DEBUG
    priv->print_perf = 1;
    priv->print_index = 1;
    priv->start_index = 0;
    priv->stop_index = 100;
    printf("init gpu:...");
#endif

    if ((priv->mode == 1 && !gpu_init (task)) || (priv->mode == 0 && !gpu_init_mode0 (task))) {
        g_set_error (error, UFO_TASK_ERROR, UFO_TASK_ERROR_SETUP, "Could not initialize GPU");
        priv->error = 1;
        return;
    }

#ifdef DEBUG
    printf("done\n");
#endif

    pcilib_init_for_transfer (priv);
    
    if(verify_streaming(priv)==FALSE){
       g_set_error (error, UFO_TASK_ERROR, UFO_TASK_ERROR_SETUP, "directgma only works with streaming enabled firmwareof ALPS platorm");
       priv->error = 1;
       return;
    } 

    get_board_generation (priv);


    if (!pcie_test (priv->bar)) {
        g_set_error (error, UFO_TASK_ERROR, UFO_TASK_ERROR_SETUP, "pcie test failed");
        priv->error = 1;
        return;
    }

    dma_conf (priv);

    if (priv->mode == 1) {
        writing_dma_descriptors (priv);
        start_dma(priv);
    }
}

static gboolean
ufo_direct_gma_task_generate (UfoTask *task,
                              UfoBuffer *output,
			                  UfoRequisition *requisition)
{
    struct timeval start;
    struct timeval end;
    static guint ok = 0;
    gfloat perf_counter;
    guint buffers_completed;
    UfoDirectGmaTaskPrivate *priv;

    priv = UFO_DIRECT_GMA_TASK_GET_PRIVATE (task);

    if (priv->error == 1)
        return FALSE;

    if (ok == priv->iterations)
        return FALSE;

    if (priv->mode == 1) {
        gpu_init_for_output (&output, priv);
        gettimeofday (&start, NULL);
        buffers_completed = handshaking_dma (output, priv);
    }
    else {
        gpu_init_for_output_mode0 (&output,priv);
        writing_dma_descriptors (priv);
        start_dma (priv);
        gettimeofday (&start, NULL);
        buffers_completed = handshaking_dma_mode0 (output, priv);
    }

    stop_dma (&end, &perf_counter, priv->bar);

#ifdef DEBUG
    printf ("transfer finished \n");
#endif

    if (priv->print_perf == 1)
        perf (start, end, perf_counter, priv, buffers_completed);

    if (priv->print_counter == 1 || priv->print_index == 1)
        print_results (output, priv);

    if (priv->counter == 1)
        ufo_buffer_convert (output,UFO_BUFFER_DEPTH_32S);

    ufo_buffer_get_device_array (output, priv->command_queue);
    ok++;

#ifdef DEBUG
    printf("giving exec to next task\n");
#endif

    return TRUE;
}

static void
ufo_direct_gma_task_set_property (GObject *object,
                                  guint property_id,
                                  const GValue *value,
                                  GParamSpec *pspec)
{
    UfoDirectGmaTaskPrivate *priv = UFO_DIRECT_GMA_TASK_GET_PRIVATE (object);

    switch (property_id) {
        case PROP_HUGE_PAGE:
            priv->huge_page = g_value_get_uint(value);
            break;
        case PROP_TLP_SIZE:
            {
                guint size;
                size = g_value_get_uint (value);
                if (size != 32 && size !=64)
                    g_warning("tlp size can be 32 or 64,and must be correct according the results of lspci command");
                else
                    priv->tlp_size = size;
            }
            break;
        case PROP_MULTIPLE:
            priv->multiple = g_value_get_uint(value);
            break;
        case PROP_WIDTH:
            priv->width = g_value_get_uint(value);
            break;
        case PROP_HEIGHT:
            priv->height = g_value_get_uint(value);
            break;
        case PROP_FRAMES:
            priv->frames = g_value_get_uint(value);
            break;
        case PROP_BUFFERS:
            priv->buffers = g_value_get_uint(value);
            break;
        case PROP_COUNTER:
            priv->counter = g_value_get_uint(value);
            break;
        case PROP_START_INDEX:
            priv->start_index = g_value_get_uint64(value);
            break;
        case PROP_STOP_INDEX:
            priv->stop_index = g_value_get_uint64(value);
            break;
        case PROP_PRINT_PERF:
            priv->print_perf = g_value_get_uint(value);
            break;
        case PROP_PRINT_INDEX:
            priv->print_index = g_value_get_uint(value);
            break;
        case PROP_PRINT_COUNTER:
            priv->print_counter = g_value_get_uint(value);
            break;
        case PROP_GET_AP_SIZE:
            priv->get_ap_size = g_value_get_uint(value);
            break;
        case PROP_ITERATIONS:
            priv->iterations = g_value_get_uint(value);
            break;
        case PROP_NB_FRAMES:
            priv->nb_frames = g_value_get_uint(value);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
            break;
    }
}

static void
ufo_direct_gma_task_get_property (GObject *object,
                              guint property_id,
                              GValue *value,
                              GParamSpec *pspec)
{
    UfoDirectGmaTaskPrivate *priv = UFO_DIRECT_GMA_TASK_GET_PRIVATE (object);

    switch (property_id) {
        case PROP_HUGE_PAGE:
            g_value_set_uint (value,priv->huge_page);
            break;
        case PROP_TLP_SIZE:
            g_value_set_uint (value,priv->tlp_size);
            break;
        case PROP_MULTIPLE:
            g_value_set_uint (value,priv->multiple);
            break;
        case PROP_HEIGHT:
            g_value_set_uint (value, priv->height);
            break;
        case PROP_WIDTH:
            g_value_set_uint (value,priv->width);
            break;
        case PROP_FRAMES:
            g_value_set_uint (value,priv->frames);
            break;
        case PROP_BUFFERS:
            g_value_set_uint (value,priv->buffers);
            break;
        case PROP_STOP_INDEX:
            g_value_set_uint64 (value,priv->stop_index);
            break;
        case PROP_START_INDEX:
            g_value_set_uint64 (value,priv->start_index);
            break;
        case PROP_COUNTER:
            g_value_set_uint (value,priv->counter);
            break;
        case PROP_PRINT_PERF:
            g_value_set_uint (value,priv->print_perf);
            break;
        case PROP_PRINT_INDEX:
            g_value_set_uint (value,priv->print_index);
            break;
        case PROP_NB_FRAMES:
            g_value_set_uint (value,priv->nb_frames);
            break;
        case PROP_PRINT_COUNTER:
            g_value_set_uint (value,priv->print_counter);
            break;
        case PROP_ITERATIONS:
            g_value_set_uint (value,priv->iterations);
            break;
        case PROP_GET_AP_SIZE:
            g_value_set_uint (value,priv->get_ap_size);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
            break;
    }
}

static void
ufo_direct_gma_task_finalize (GObject *object)
{
  UfoDirectGmaTaskPrivate* priv;
  priv = UFO_DIRECT_GMA_TASK_GET_PRIVATE(object);

  if (priv->mode == 1)
      free_and_close (priv);
  else
      free_and_close_mode0 (priv);

  G_OBJECT_CLASS (ufo_direct_gma_task_parent_class)->finalize (object);
}

static void
ufo_task_interface_init (UfoTaskIface *iface)
{
    iface->setup = ufo_direct_gma_task_setup;
    iface->get_num_inputs = ufo_direct_gma_task_get_num_inputs;
    iface->get_num_dimensions = ufo_direct_gma_task_get_num_dimensions;
    iface->get_mode = ufo_direct_gma_task_get_mode;
    iface->get_requisition = ufo_direct_gma_task_get_requisition;
    iface->generate = ufo_direct_gma_task_generate;
}

static void
ufo_direct_gma_task_class_init (UfoDirectGmaTaskClass *klass)
{
    GObjectClass *oclass = G_OBJECT_CLASS (klass);

    oclass->set_property = ufo_direct_gma_task_set_property;
    oclass->get_property = ufo_direct_gma_task_get_property;
    oclass->finalize = ufo_direct_gma_task_finalize;

    properties[PROP_HUGE_PAGE]=
        g_param_spec_uint("huge-page",
			  "number of pages of 4k in one dma buffer",
			  "number of pages of 4k in one dma buffer",
			  1, 2 << 16, 1200,
			  G_PARAM_READWRITE);

    properties[PROP_MULTIPLE]=
        g_param_spec_uint("multiple",
			  "represents the number of virtual buffers used for dma",
			  "represents the number of virtual buffers used for dma",
			  1, 2 << 16, 2,
              G_PARAM_READWRITE);

    properties[PROP_TLP_SIZE]=
        g_param_spec_uint("tlp-size",
			  "size for the corresponding payload size of pcie frame",
			  "size for the corresponding payload size of pcie frame",
			  32, 64, 32,
              G_PARAM_READWRITE);

    properties[PROP_HEIGHT]=
        g_param_spec_uint("height",
			  "height of the camera frame for image processing afterwards",
			  "height of the camera frame for image processing afterwards",
			  1, G_MAXUINT, 3840,
              G_PARAM_READWRITE);

    properties[PROP_WIDTH]=
        g_param_spec_uint("width",
			  "width of the camera frame for image processing afterwards",
			  "width of the camera frame for image processing afterwards",
			  1, G_MAXUINT, 5120,
              G_PARAM_READWRITE);

    properties[PROP_FRAMES]=
        g_param_spec_uint("frames",
              "number of frames transmitted (number of times the transfer is done)",
              "number of frames transmitted (number of times the transfer is done)",
			  0, 1, 1,
              G_PARAM_READWRITE);

    properties[PROP_BUFFERS]=
        g_param_spec_uint("buffers",
              "number of buffers for directgma",
              "number of buffers for directgma",
			  2, 8000, 8,
              G_PARAM_READWRITE);

    properties[PROP_PRINT_INDEX]=
        g_param_spec_uint("print-index",
			  "indicates if we want to print results in index way",
			  "indicates if we want to print results in index way",
			  0, 1, 0,
			  G_PARAM_READWRITE);

    properties[PROP_PRINT_COUNTER]=
        g_param_spec_uint("print-counter",
			  "indicates if we want to print results of counter",
			  "indicates if we want to print results of counter",
			  0, 1, 0,
			  G_PARAM_READWRITE);

    properties[PROP_COUNTER]=
        g_param_spec_uint("counter",
              "indicates if we want to check data for counter",
              "indicates if we want to check data for counter",
			  0, 1, 0,
              G_PARAM_READWRITE);

    properties[PROP_START_INDEX]=
        g_param_spec_uint64("start-index",
              "starting index for printing results",
              "starting index for printing results",
			  0, G_MAXUINT64, 0,
              G_PARAM_READWRITE);

    properties[PROP_STOP_INDEX]=
        g_param_spec_uint64("stop-index",
                "ending index for printing results",
                "ending index for printing results",
                0,G_MAXUINT64,0,
                G_PARAM_READWRITE);

    properties[PROP_PRINT_PERF]=
        g_param_spec_uint("print-perf",
			  "parameter to print performance or not",
			  "parameter to print performance or not",
			  0, 1, 0,
			  G_PARAM_READWRITE);

    properties[PROP_GET_AP_SIZE]=
        g_param_spec_uint("get-ap-size",
			  "parameter for auto-detect of ap size or not",
			  "parameter for auto-detect of ap size or not",
			  0, 2, 0,
			  G_PARAM_READWRITE);

    properties[PROP_ITERATIONS]=
        g_param_spec_uint("iterations",
			  "number of iterations of generate function",
			  "number of iterations of generate function",
			  1, 2 << 16, 1,
			  G_PARAM_READWRITE);

    properties[PROP_NB_FRAMES]=
        g_param_spec_uint("nb-frames",
			  "number of frames",
			  "number of frames",
			  1, 80, 1,
			  G_PARAM_READWRITE);

    for (guint i = PROP_0 + 1; i < N_PROPERTIES; i++)
        g_object_class_install_property (oclass, i, properties[i]);

    g_type_class_add_private (oclass, sizeof(UfoDirectGmaTaskPrivate));
}

static void
ufo_direct_gma_task_init(UfoDirectGmaTask *self)
{
    self->priv = UFO_DIRECT_GMA_TASK_GET_PRIVATE(self);
    self->priv->tlp_size = 32;
    self->priv->huge_page = 1200;
    self->priv->multiple = 2;
    self->priv->height = 3840;
    self->priv->width = 5120;
    self->priv->frames = 1;
    self->priv->buffers = 8;
    self->priv->counter = 0;
    self->priv->stop_index = 10;
    self->priv->start_index = 0;
    self->priv->print_perf = 0;
    self->priv->print_counter = 0;
    self->priv->print_index = 0;
    self->priv->get_ap_size = 0;
    self->priv->iterations = 1;
    self->priv->nb_frames = 1;
}
