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

#define SIZE_128_M 128000000
#define SIZE_96_M 96000000

#define PAGE_SIZE       4096        // other values are not supported in the kernel

#define FPGA_CLOCK 250

#define WR(addr, value) { *(uint32_t*)(bar + addr + offset) = value; }
#define RD(addr, value) { value = *(uint32_t*)(bar + addr + offset); }

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

struct _UfoDirectGmaTaskPrivate {
    gboolean foo;
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
    volatile void* bar;
    pcilib_t *pci;
    guintptr kdesc_bus;
    volatile guint32 *desc;
    guint board_gen;
    guint get_board_gen;
    guint streaming;
    guint get_streaming;
    guint print_perf;
    guint print_index;
    guint print_counter;
    guint counter;
    guint get_ap_size;
    guint mode;
    guint iterations;
    guint error;
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
    PROP_GET_BOARD_GEN,
    PROP_GET_STREAMING,
    PROP_PRINT_PERF,
    PROP_PRINT_COUNTER,
    PROP_PRINT_INDEX,
    PROP_GET_AP_SIZE,
    PROP_ITERATIONS,
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
    requisition->dims[0]=priv->width;
    requisition->dims[1]=priv->height;
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
  return UFO_TASK_MODE_GENERATOR | UFO_TASK_MODE_GPU ;
}

static void
init_buffer_gma(UfoBuffer** buffer, cl_command_queue* command_queue, int init){
    ufo_buffer_init_gma(*buffer, &init, command_queue);
}

static void 
printf_with_index(guint start, guint stop, int* buffer){
    guint i;
    for(i=start;i<stop;i++) printf("%x |",buffer[i]);
    printf("\n");
}

static int 
verify_aperture_size(UfoDirectGmaTaskPrivate *task_priv)
{
    guint aperture_size;

    if(task_priv->get_ap_size==1){
        FILE *fp= popen("aticonfig --get-pcs-key=MCIL,DMAOGLExtensionApertureMB | grep -o '[0-9][0-9])$' | grep -o '[0-9][0-9]' ", "r");
        fscanf(fp,"%lu",&aperture_size);
        aperture_size*=1000000;
    }
    else if(task_priv->get_ap_size==0){
        aperture_size=SIZE_96_M;
    }
    else if(task_priv->get_ap_size==2){
        aperture_size=SIZE_128_M;
    }

#ifdef DEBUG
    printf("aperture size obtained: %u \n",aperture_size);
#endif
        
    if((task_priv->buffers*task_priv->huge_page*4096)>aperture_size){
        pcilib_error("the size for buffers for gma is higher than the aperture size\n");
        return 1;
    }
    
    if((task_priv->width*task_priv->height*4)>aperture_size) task_priv->mode=1;
    else task_priv->mode=0;
    
#ifdef DEBUG
    printf("mode obtained : %i\n", task_priv->mode);
#endif
    return 0;
}

static glong 
create_gma_buffer(UfoBuffer** buffer,UfoDirectGmaTaskPrivate *task_priv,cl_bus_address_amd* busadress, cl_command_queue *command_queue){

    *buffer=(UfoBuffer*)ufo_buffer_new_with_size_in_bytes(1024*task_priv->huge_page*sizeof(int), task_priv->context);
    ufo_buffer_set_location(*buffer,UFO_BUFFER_LOCATION_DEVICE_DIRECT_GMA);
    ufo_buffer_get_device_array_for_directgma(*buffer,command_queue,task_priv->platform_id,busadress);

    return busadress->surface_bus_address;
}

static void 
get_board_generation(UfoDirectGmaTaskPrivate *task_priv)
{

    if(task_priv->get_board_gen==1){
        FILE *fp2= popen("pci -r 0x18 | grep -o '[0-9][0-9][0-9]$' ", "r");
        fscanf(fp2,"%i",&(task_priv->board_gen));
    }else if(task_priv->get_board_gen==0){
	  task_priv->board_gen=3;
    }else if(task_priv->get_board_gen==2){
	  task_priv->board_gen=2;
    }
}

static void 
get_streaming(UfoDirectGmaTaskPrivate *task_priv)
{

    if(task_priv->get_streaming==1){
        FILE *fp3= popen("pci -r 0x18 | grep -o '^[0-9]' ", "r");
        fscanf(fp3,"%i",&(task_priv->streaming));
    }else if(task_priv->get_streaming==0){
	  task_priv->streaming=1;
    }else if(task_priv->get_streaming==2){
	  task_priv->streaming=0;
    }
}

static int
gpu_init(UfoTask* task)
{
    GTimer *timer = g_timer_new ();
  cl_bus_address_amd* busadresses; 
  guint i;
    UfoGpuNode *node;
    UfoDirectGmaTaskPrivate *task_priv;
    task_priv= UFO_DIRECT_GMA_TASK_GET_PRIVATE(task); 
    
    node = UFO_GPU_NODE (ufo_task_node_get_proc_node (UFO_TASK_NODE (task)));
    task_priv->command_queue = ufo_gpu_node_get_cmd_queue (node);
   
    busadresses=malloc(task_priv->buffers*sizeof(cl_bus_address_amd));
    if((task_priv->buffers*task_priv->multiple*task_priv->huge_page)>1048576){
      pcilib_error("the total size is too big\n");
      return 1;
    }

    if((task_priv->buffers*task_priv->huge_page*4096)>96000000){
      pcilib_error("the size for buffers for gma is higher than the aperture size\n");
      return 1;
    }

#ifdef DEBUG
    int* results;
    results=malloc(task_priv->huge_page*task_priv->buffers*1024*sizeof(int));
#endif
    for(i=0;i<task_priv->buffers;i++){
      task_priv->buffer_gma_addr[i]=create_gma_buffer(&(task_priv->buffers_gma[i]),task_priv,&busadresses[i],&(task_priv->command_queue));

#ifndef PERF_MAX
      init_buffer_gma(&(task_priv->buffers_gma[i]), &(task_priv->command_queue),42);
#endif

#ifdef DEBUG
      ufo_buffer_read(task_priv->buffers_amd[i],results,&(task_priv->command_queue));
    printf("\n buffer directgma %i\n",i);
    if(task_priv->print_index==1) printf_with_index(task_priv->start_index,task_priv->stop_index,results);
#endif
    	if (task_priv->buffer_gma_addr[i]==0){
	  pcilib_error("the buffer %i for directgma has not been allocated correctly\n");
	  return 1;
	}
    }
#ifdef DEBUG
    free(results);
#endif
    g_timer_stop (timer);
    g_print ("gpuinit=%fs\n", g_timer_elapsed (timer, NULL));
    g_timer_destroy (timer);
    return 0;
}    

static int
gpu_init_mode0(UfoTask* task){
    UfoGpuNode *node;
    UfoDirectGmaTaskPrivate *task_priv;

    task_priv= UFO_DIRECT_GMA_TASK_GET_PRIVATE(task); 
    node = UFO_GPU_NODE (ufo_task_node_get_proc_node (UFO_TASK_NODE (task)));
    task_priv->command_queue = ufo_gpu_node_get_cmd_queue (node);
   
    if((task_priv->buffers*task_priv->multiple*task_priv->huge_page)>1048576){
        pcilib_error("the total size is too big\n");
        return 1;
    }
    return 0;
}


static void
gpu_init_for_output( UfoBuffer **saving_buffers, UfoDirectGmaTaskPrivate* task_priv)
{
    ufo_buffer_set_location(*saving_buffers, UFO_BUFFER_LOCATION_DEVICE);
    ufo_buffer_get_device_array(*saving_buffers,&(task_priv->command_queue));

#ifndef PERF_MAX
    init_buffer_gma(saving_buffers,&(task_priv->command_queue), 666);
#endif

#ifdef DEBUG
    int* results;
    results=malloc(task_priv->multiple*task_priv->huge_page*task_priv->buffers*1024*sizeof(int));
    printf("final buffer\n");
    ufo_buffer_read(*saving_buffers,results,&(task_priv->command_queue));
    if(task_priv->print_index==1) printf_with_index(task_priv->start_index,task_priv->stop_index,results);
    free(results);
#endif
}

static void
gpu_init_for_output_mode0( UfoBuffer **saving_buffers, cl_command_queue* command_queue, UfoDirectGmaTaskPrivate* task_priv)
  {
guint j;
cl_bus_address_amd busaddress;

    ufo_buffer_set_location(*saving_buffers, UFO_BUFFER_LOCATION_DEVICE_DIRECT_GMA);
    ufo_buffer_get_device_array_for_directgma(*saving_buffers,command_queue,task_priv->platform_id,&busaddress);

#ifndef PERF_MAX
    init_buffer_gma(saving_buffers, command_queue, 69);
#endif

#ifdef DEBUG
    int* results;
    results=malloc(task_priv->multiple*task_priv->huge_page*task_priv->buffers*1024*sizeof(int));
    printf("final buffer\n");
    ufo_buffer_read(*saving_buffers,results,command_queue);
    if(task_priv->print_index==1) printf_with_index(task_priv->start_index,task_priv->stop_index,results);
    free(results);
#endif

    task_priv->buffer_gma_addr[0]=busaddress.surface_bus_address;
#ifdef DEBUG
      printf("gma buffer 0 addr: %lu  \n",task_priv->buffer_gma_addr[0]);
#endif

    for(j=1;j<task_priv->buffers;j++){
      task_priv->buffer_gma_addr[j]=task_priv->buffer_gma_addr[j-1]+4096*task_priv->huge_page;
#ifdef DEBUG
      printf("gma buffer %j addr: %lu\n",j,task_priv->buffer_gma_addr[j]);
#endif
    }
}

static gint
pcie_test(volatile void* bar){
    guintptr offset=0;
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
printf("\xE2\x9C\x93 \n");
#endif
       return 0;
    } else {
       printf("PCIe not ready!\n");
       return 1;
    }
}


static void
dma_conf(UfoDirectGmaTaskPrivate* task_priv){
    guintptr offset=0;
    volatile void* bar=task_priv->bar;
    
#ifdef DEBUG
    printf("DMA: send data mount\n");
#endif
    WR(NB_PACKETS_BY_DESC, (task_priv->huge_page * (PAGE_SIZE / (4 * task_priv->tlp_size))));                              
#ifdef DEBUG
    printf("DMA: putting running mode\n");
#endif
    if(task_priv->board_gen==3){
      if (task_priv->tlp_size == 64)
	{
	  WR(PACKET_PARAM, 0x80040);
	}
      else if (task_priv->tlp_size == 32)
	{
	  WR(PACKET_PARAM, 0x80020);
	}
    }else{
      if (task_priv->tlp_size == 64) 
	{
	  WR(PACKET_PARAM, 0x0040);
	}
      else if (task_priv->tlp_size == 32) 
	{
	  WR(PACKET_PARAM, 0x0020);
	}
    }
    
    WR(NB_DESCRIPTORS_FPGA, 0x00); 
}

static void
pcilib_init_for_transfer(UfoDirectGmaTaskPrivate* task_priv){
    pcilib_kmem_handle_t *kdesc;
    pcilib_kmem_flags_t flags = PCILIB_KMEM_FLAG_HARDWARE|PCILIB_KMEM_FLAG_PERSISTENT|PCILIB_KMEM_FLAG_EXCLUSIVE;
    pcilib_kmem_flags_t clean_flags = PCILIB_KMEM_FLAG_HARDWARE|PCILIB_KMEM_FLAG_PERSISTENT|PCILIB_KMEM_FLAG_EXCLUSIVE;
    pcilib_bar_t bar_tmp = BAR;
    uintptr_t offset = 0;

    (task_priv->pci) = pcilib_open(DEVICE, "pci");
    if (!(task_priv->pci)) pcilib_error("pcilib_open");
  
    task_priv->bar = pcilib_map_bar(task_priv->pci, BAR);
    if (!(task_priv->bar)) {
       pcilib_close(task_priv->pci);
       pcilib_error("map bar");
    }

    pcilib_detect_address(task_priv->pci, &bar_tmp, &offset, 1);

    pcilib_enable_irq(task_priv->pci, PCILIB_IRQ_TYPE_ALL, 0);
    pcilib_clear_irq(task_priv->pci, PCILIB_IRQ_SOURCE_DEFAULT);

    pcilib_clean_kernel_memory(task_priv->pci, USE, clean_flags);
    pcilib_clean_kernel_memory(task_priv->pci, USE_RING, clean_flags);

    kdesc = pcilib_alloc_kernel_memory(task_priv->pci, PCILIB_KMEM_TYPE_CONSISTENT, 1, 128, 4096, USE_RING, flags);
    task_priv->kdesc_bus = pcilib_kmem_get_block_ba(task_priv->pci, kdesc, 0);
    task_priv->desc = (uint32_t*)pcilib_kmem_get_block_ua(task_priv->pci, kdesc, 0);
    memset((void*)task_priv->desc, 0, 5*sizeof(uint32_t));
#ifdef DEBUG
    printf("bar debut: %p\n",bar);
#endif
}

/*
static void
writing_dma_descriptors(glong* buffer_gma_addr,uintptr_t kdesc_bus,volatile void* bar, uintptr_t *bus_addr, UfoDirectGmaTaskPrivate* task_priv){
    uintptr_t offset = 0;
    guint j;
    //    bar=task_priv->bar;

#ifdef DEBUG
    printf("Writing SW Read Descriptor\n");
    printf("bar adress: %p\n",bar);
    printf("nb buffers: %i\n",task_priv->buffers);
#endif

    WR(LAST_REG_READ_DRIVER, task_priv->buffers-1);

#ifdef DEBUG
    printf("Writing the Descriptor Threshold\n");
#endif

    WR(THRESHOLD_REG, DESC_THRESHOLD);

    WR(UPDATE_REG, kdesc_bus);
    usleep(100000);
    for (j = 0; j < task_priv->buffers; j++ ) {
      bus_addr[j]=buffer_gma_addr[j];
        usleep(1000);
#ifdef DEBUG
    printf("Writing descriptor num. %i: \t %08lx \n", j, bus_addr[j]);
#endif

        WR(DESCRIPTOR_MEMORY, bus_addr[j]);
    }


    }*/
static void
writing_dma_descriptors(UfoDirectGmaTaskPrivate* task_priv){
    uintptr_t offset = 0;
    guint j;
    volatile void* bar=task_priv->bar;

#ifdef DEBUG
    printf("Writing SW Read Descriptor\n");
    printf("bar adress: %p\n",bar);
    printf("nb buffers: %i\n",task_priv->buffers);
#endif

    WR(LAST_REG_READ_DRIVER, task_priv->buffers-1);

#ifdef DEBUG
    printf("Writing the Descriptor Threshold\n");
#endif

    WR(THRESHOLD_REG, DESC_THRESHOLD);

    WR(UPDATE_REG, task_priv->kdesc_bus);
    usleep(100000);
    for (j = 0; j < task_priv->buffers; j++ ) {
      task_priv->bus_addr[j]=task_priv->buffer_gma_addr[j];
        usleep(1000);
#ifdef DEBUG
    printf("Writing descriptor num. %i: \t %08lx \n", j, task_priv->bus_addr[j]);
#endif

        WR(DESCRIPTOR_MEMORY, task_priv->bus_addr[j]);
    }


}


static void
handshaking_dma(UfoBuffer** buffers_gma, UfoBuffer* saving_buffers, volatile uint32_t *desc,volatile void* bar, cl_command_queue* command_queue , uintptr_t* bus_addr, UfoDirectGmaTaskPrivate* task_priv, guint* buffers_completed){
 guint i;
 uintptr_t offset = 0;
 guint32 curptr,hwptr, curbuf;
 gint err;

#ifdef DEBUG2
 int* results;
 results=malloc(task_priv->huge_page*task_priv->buffers*1024*sizeof(int));
#endif
 i=0;
 curptr=0;
 curbuf=0;
 GTimer *timer2 = g_timer_new ();

 while (i < task_priv->multiple) {
     do {
         if(task_priv->board_gen==3)
             hwptr = desc[3];
         else
             hwptr = desc[4];
     } while (hwptr == curptr);

     do {    
         err=ufo_buffer_copy_for_directgma(buffers_gma[curbuf],saving_buffers,(i*task_priv->buffers+curbuf),command_queue);
#ifdef DEBUG2
         printf("loop %i with curbuf %i\n",i,curbuf);
         ufo_buffer_read(buffers_gma[curbuf],results,command_queue);
         if(task_priv->print_index==1) printf_with_index(task_priv->start_index,task_priv->stop_index,results);
#endif
         if(err==-30) break;
         if(task_priv->streaming==1){
             if (i < (task_priv->multiple-1) || (i==(task_priv->multiple-1) && curbuf<1))
                 if (desc[1] == 0)
                     WR(DESCRIPTOR_MEMORY, bus_addr[curbuf]);
         }

         curbuf++;
         if (curbuf == task_priv->buffers) {
             i++;
             curbuf = 0;
             if (i >= task_priv->multiple) break;
         }
     } while (bus_addr[curbuf] != hwptr);

     if(err==-30) break;
     if(task_priv->board_gen==3){                 
         if (desc[1] != 0){
             err=ufo_buffer_copy_for_directgma(buffers_gma[curbuf],saving_buffers,(i*task_priv->buffers+curbuf),command_queue);
             break;
         }
     }else {   
         if (desc[2] != 0){
             if (bus_addr[curbuf] == hwptr) {
                 break;
             }
         }
     }
     curptr = hwptr;
 }

 g_timer_stop (timer2);
 g_print ("transfer: %fs\n", g_timer_elapsed (timer2, NULL));
 g_timer_destroy (timer2);

 if(curbuf!=0) *buffers_completed=i*task_priv->buffers+curbuf;
 else *buffers_completed=i*task_priv->buffers+curbuf-1;
#ifdef DEBUG2
 free(results);
#endif
}

static void
handshaking_dma_mode0(UfoBuffer* saving_buffers, volatile uint32_t *desc,volatile void* bar, cl_command_queue* command_queue , uintptr_t* bus_addr, UfoDirectGmaTaskPrivate* task_priv, guint* buffers_completed){
 guint i;
 uintptr_t offset = 0;
 guint32 curptr,hwptr, curbuf;

    i=0;
    curptr=0;
    curbuf=0;
    GTimer *timer = g_timer_new ();
    while (i < 2) {
        do {
	  if(task_priv->board_gen==3)
                hwptr = desc[3];
	  else
                hwptr = desc[4];
        } while (hwptr == curptr);

        do {    

	if(task_priv->streaming==1){
	  if (curbuf<1)
	    if (desc[1] == 0)
              WR(DESCRIPTOR_MEMORY, bus_addr[curbuf]);
	}
            
            curbuf++;
            if (curbuf == task_priv->buffers) {
                i++;
                curbuf = 0;
                if (i >= task_priv->multiple) break;
            }
        } while (bus_addr[curbuf] != hwptr);
	
	if(task_priv->board_gen==3){                 
	  if (desc[1] != 0){
	      break;
	  }
	}else {   
	    if (desc[2] != 0){
	      if (bus_addr[curbuf] == hwptr) {
		break;
	      }
	    }
	}
	  curptr = hwptr;
    }

  g_timer_stop (timer);
  g_print ("transfer: %fs\n", g_timer_elapsed (timer, NULL));
  g_timer_destroy (timer);
  if(curbuf!=0) *buffers_completed=curbuf;
  else *buffers_completed=curbuf-1;
}

static void
stop_dma(struct timeval *end,gfloat* perf_counter,volatile void* bar){
 uintptr_t offset = 0;
#ifdef DEBUG
 printf("dma stop\n");
#endif
    gettimeofday(end, NULL);
    WR(DMA_START, 0x00);
    usleep(100);
    RD(PERF_COUNTER, *perf_counter);
    usleep(100);
    WR(RESET_DMA, 0x01);
}

static void
free_and_close(pcilib_t *pci,volatile void* bar,UfoBuffer **buffers_gma, uintptr_t* bus_addr, UfoDirectGmaTaskPrivate* task_priv, glong* addresses){
  guint j;
    free(bus_addr);
    for(j=0;j<task_priv->buffers;j++)
        g_object_unref(buffers_gma[j]);
    free(addresses);
    pcilib_disable_irq(pci, 0);
    pcilib_unmap_bar(pci, BAR, bar);
    pcilib_close(pci);
}

static void
free_and_close_mode0(pcilib_t *pci,volatile void* bar, uintptr_t* bus_addr, UfoDirectGmaTaskPrivate* task_priv, glong* addresses){
    free(bus_addr);
    free(addresses);
    pcilib_disable_irq(pci, 0);
    pcilib_unmap_bar(pci, BAR, bar);
    pcilib_close(pci);
}



static void
start_dma(volatile void* bar, UfoDirectGmaTaskPrivate* task_priv){
     guintptr offset=0;
     gint nb_frames;
     gint nb_rows;
#ifdef DEBUG
     printf("putting data generator\n");
     printf("frames %i counter %i\n",task_priv->frames, task_priv->counter);
#endif
    if(task_priv->frames==1){
      nb_frames=task_priv->height/3840;
      nb_rows=3840;
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
     else if (task_priv->counter==1){
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

static void
perf( struct timeval start, struct timeval end, float perf_counter, UfoDirectGmaTaskPrivate* task_priv, guint buffers_completed){

    gfloat performance;
    gsize run_time;
    gfloat size_mb;

     run_time = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
    size_mb=buffers_completed*task_priv->huge_page/256;
     printf("Performance: transfered %f Mbytes in %zu us using %d buffers\n", (size_mb), run_time, task_priv->buffers);
     performance = ((size_mb * FPGA_CLOCK * 1000000)/(perf_counter*256));
     printf("DMA perf counter:\t%d\n", (int)perf_counter); 
     printf("DMA side:\t\t%.3lf MB/s\n", performance);  
     printf("PC side:\t\t%.3lf MB/s\n\n", 1000000. * size_mb / run_time );
}

static void
research_data_fail_counter(int* buffer, UfoDirectGmaTaskPrivate* task_priv){
    guint i;
    guint k=0;
    for(i=1;i<task_priv->multiple*task_priv->huge_page*task_priv->buffers*1024-1;i++){
        if(buffer[i+1]-buffer[i]!=1){
            printf("problem at position %i: %i %i %i %i\n",i, buffer[i-1],buffer[i], buffer[i+1],buffer[i+2]);
	    k++;
        }
    }
    
    if(k==0) printf("no problem in data\n");
}

static void
print_results(cl_command_queue* cmd_queue, UfoBuffer* buffer, UfoDirectGmaTaskPrivate* task_priv){
    int* results;
    results=malloc(task_priv->multiple*task_priv->huge_page*task_priv->buffers*1024*sizeof(int));
    printf("results: \n");
    ufo_buffer_read(buffer,results,cmd_queue);
    if(task_priv->print_counter==1) research_data_fail_counter(results, task_priv);
    if(task_priv->print_index==1) printf_with_index(task_priv->start_index,task_priv->stop_index,results);
    free(results);
}

static void
ufo_direct_gma_task_setup (UfoTask *task,
			   UfoResources *resources,
			   GError **error)
{
    UfoDirectGmaTaskPrivate *task_priv;
    task_priv = UFO_DIRECT_GMA_TASK_GET_PRIVATE (task);
    gint err;
#ifdef DEBUG
    printf("setup start\n");
#endif
    task_priv->context= ufo_resources_get_context(resources);
    ufo_get_platform_id_for_directgma(resources, &(task_priv->platform_id));
    
    if((err=verify_aperture_size(task_priv))==1){
      task_priv->error=1;
      return;
    }
    
    task_priv->buffer_gma_addr=malloc(task_priv->buffers*sizeof(glong));
    task_priv->buffers_gma=malloc(task_priv->buffers*sizeof(UfoBuffer*));
    task_priv->bus_addr=malloc(task_priv->buffers*sizeof(uintptr_t));
    get_board_generation(task_priv);
    get_streaming(task_priv);

#ifdef DEBUG
    task_priv->print_perf=1;
    task_priv->print_index=1;
    task_priv->start_index=0;
    task_priv->stop_index=100;
    printf("init gpu:...");
#endif

    if(task_priv->mode==1){
      if((err=gpu_init(task))==1){
	task_priv->error=1;
	return;
      }
    }else{
      printf("mode 0\n");
      gpu_init_mode0(task);
    }

#ifdef DEBUG
    printf("done\n");
#endif

    pcilib_init_for_transfer(task_priv);
    if((err=pcie_test(task_priv->bar))==1){
      task_priv->error=1;
      return;
    }
    dma_conf(task_priv);

    if(task_priv->mode==1){
      //    writing_dma_descriptors(task_priv->buffer_gma_addr, task_priv->kdesc_bus,task_priv->bar, task_priv->bus_addr, task_priv);
      writing_dma_descriptors(task_priv);
      start_dma(task_priv->bar,task_priv);
    }

}

static gboolean
ufo_direct_gma_task_generate (UfoTask *task,
                              UfoBuffer *output,
			      UfoRequisition *requisition)
{
    struct timeval start;
    struct timeval end;
    static guint ok=0;
    gfloat perf_counter;
    guint buffers_completed;
    UfoDirectGmaTaskPrivate *task_priv;
    cl_mem foo;
    gchar baz;

    task_priv= UFO_DIRECT_GMA_TASK_GET_PRIVATE(task);

    if(task_priv->error==1) return FALSE;;
    if(ok==task_priv->iterations) return FALSE;   

    if(task_priv->mode==1){

      gpu_init_for_output(&output,task_priv);

      foo = ufo_buffer_get_device_array (output, task_priv->command_queue);
      clEnqueueReadBuffer (task_priv->command_queue, foo, CL_TRUE, 0, 1, &baz, 0, NULL, NULL);

      gettimeofday(&start,NULL);
      handshaking_dma(task_priv->buffers_gma, output, task_priv->desc,task_priv->bar,&(task_priv->command_queue),task_priv->bus_addr,task_priv, &buffers_completed);
    }else{
       gpu_init_for_output_mode0(&output,&(task_priv->command_queue),task_priv);
       //writing_dma_descriptors(task_priv->buffer_gma_addr, task_priv->kdesc_bus,task_priv->bar, task_priv->bus_addr, task_priv);
       writing_dma_descriptors(task_priv);
       start_dma(task_priv->bar,task_priv);
       handshaking_dma_mode0(output, task_priv->desc,task_priv->bar, &(task_priv->command_queue), task_priv->bus_addr, task_priv, &buffers_completed);
    }
      
    stop_dma(&end, &perf_counter,task_priv->bar);

#ifdef DEBUG
    printf("transfer finished \n");
#endif

    if(task_priv->print_perf==1) perf(start,end,perf_counter, task_priv, buffers_completed);
    if(task_priv->print_counter==1 || task_priv->print_index==1) print_results(&(task_priv->command_queue),output, task_priv);

#ifndef PERF_MAX
    if(task_priv->mode==1) free_and_close(task_priv->pci,task_priv->bar,task_priv->buffers_gma,task_priv->bus_addr,task_priv,task_priv->buffer_gma_addr);
    else free_and_close_mode0(task_priv->pci,task_priv->bar,task_priv->bus_addr,task_priv,task_priv->buffer_gma_addr);
#endif

    if(task_priv->counter==1) ufo_buffer_convert(output,UFO_BUFFER_DEPTH_32S);

    ufo_buffer_get_device_array(output,task_priv->command_queue);
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
	    priv->huge_page=g_value_get_uint(value);
            break;
        case PROP_TLP_SIZE:
            {
	        guint size;
                size=g_value_get_uint(value);
                if(size != 32 && size !=64)
                    g_warning("tlp size can be 32 or 64,and must be correct according the results of lspci command");
                else
	            priv->tlp_size=size;
            }
            break;
        case PROP_MULTIPLE:
            priv->multiple=g_value_get_uint(value);
            break;
        case PROP_WIDTH:
            priv->width=g_value_get_uint(value);
            break;
        case PROP_HEIGHT:
            priv->height=g_value_get_uint(value);
            break;
        case PROP_FRAMES:
            priv->frames=g_value_get_uint(value);
            break;
        case PROP_BUFFERS:
            priv->buffers=g_value_get_uint(value);
            break;
        case PROP_COUNTER:
            priv->counter=g_value_get_uint(value);
            break;
        case PROP_START_INDEX:
            priv->start_index=g_value_get_uint64(value);
            break;
        case PROP_STOP_INDEX:
            priv->stop_index=g_value_get_uint64(value);
            break;
        case PROP_GET_BOARD_GEN:
            priv->get_board_gen=g_value_get_uint(value);
            break;
        case PROP_GET_STREAMING:
            priv->get_streaming=g_value_get_uint(value);
            break;
        case PROP_PRINT_PERF:
            priv->print_perf=g_value_get_uint(value);
            break;
        case PROP_PRINT_INDEX:
            priv->print_index=g_value_get_uint(value);
            break;
        case PROP_PRINT_COUNTER:
            priv->print_counter=g_value_get_uint(value);
            break;
        case PROP_GET_AP_SIZE:
            priv->get_ap_size=g_value_get_uint(value);
            break;
        case PROP_ITERATIONS:
            priv->iterations=g_value_get_uint(value);
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
	    g_value_set_uint(value,priv->huge_page);
            break;
        case PROP_TLP_SIZE:
            g_value_set_uint(value,priv->tlp_size);
            break;
        case PROP_MULTIPLE:
            g_value_set_uint(value,priv->multiple);
            break;
        case PROP_HEIGHT:
            g_value_set_uint(value, priv->height);
            break;
        case PROP_WIDTH:
            g_value_set_uint(value,priv->width);
            break;
        case PROP_FRAMES:
            g_value_set_uint(value,priv->frames);
            break;
        case PROP_BUFFERS:
            g_value_set_uint(value,priv->buffers);
            break;
        case PROP_STOP_INDEX:
            g_value_set_uint64(value,priv->stop_index);
            break;
        case PROP_START_INDEX:
            g_value_set_uint64(value,priv->start_index);
            break;
        case PROP_COUNTER:
            g_value_set_uint(value,priv->counter);
            break;
        case PROP_GET_BOARD_GEN:
            g_value_set_uint(value,priv->get_board_gen);
            break;
        case PROP_GET_STREAMING:
            g_value_set_uint(value,priv->get_streaming);
            break;
        case PROP_PRINT_PERF:
            g_value_set_uint(value,priv->print_perf);
            break;
        case PROP_PRINT_INDEX:
            g_value_set_uint(value,priv->print_index);
            break;
        case PROP_PRINT_COUNTER:
            g_value_set_uint(value,priv->print_counter);
            break;
        case PROP_ITERATIONS:
            g_value_set_uint(value,priv->iterations);
            break;
        case PROP_GET_AP_SIZE:
            g_value_set_uint(value,priv->get_ap_size);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
            break;
    }
}

static void
ufo_direct_gma_task_finalize (GObject *object)
{
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
			  1,2<<16,1000,
			  G_PARAM_READWRITE);

    properties[PROP_MULTIPLE]=
        g_param_spec_uint("multiple",
			  "represents the number of virtual buffers used for dma",
			  "represents the number of virtual buffers used for dma",
			  1,2<<16,2,
              G_PARAM_READWRITE);

    properties[PROP_TLP_SIZE]=
        g_param_spec_uint("tlp-size",
			  "size for the corresponding payload size of pcie frame",
			  "size for the corresponding payload size of pcie frame",
			  32,64,32,
              G_PARAM_READWRITE);

    properties[PROP_HEIGHT]=
        g_param_spec_uint("height",
			  "height of the camera frame for image processing afterwards",
			  "height of the camera frame for image processing afterwards",
			  1,G_MAXUINT,8192,
              G_PARAM_READWRITE);
    
    properties[PROP_WIDTH]=
        g_param_spec_uint("width",
			  "width of the camera frame for image processing afterwards",
			  "width of the camera frame for image processing afterwards",
			  1,2<<16,8000,
              G_PARAM_READWRITE);

    properties[PROP_FRAMES]=
        g_param_spec_uint("frames",
              "number of frames transmitted (number of times the transfer is done)",
              "number of frames transmitted (number of times the transfer is done)",
			  0,1,1,
              G_PARAM_READWRITE);

    properties[PROP_BUFFERS]=
        g_param_spec_uint("buffers",
              "number of buffers for directgma",
              "number of buffers for directgma",
			  2,8000,8,
              G_PARAM_READWRITE);

    properties[PROP_PRINT_INDEX]=
        g_param_spec_uint("print-index",
			  "indicates if we want to print results in index way",
			  "indicates if we want to print results in index way",
			  0,1,0,
			  G_PARAM_READWRITE);

    properties[PROP_PRINT_COUNTER]=
        g_param_spec_uint("print-counter",
			  "indicates if we want to print results of counter",
			  "indicates if we want to print results of counter",
			  0,1,0,
			  G_PARAM_READWRITE);

    properties[PROP_COUNTER]=
        g_param_spec_uint("counter",
              "indicates if we want to check data for counter",
              "indicates if we want to check data for counter",
			  0,1,0,
              G_PARAM_READWRITE);

    properties[PROP_START_INDEX]=
        g_param_spec_uint64("start-index",
              "starting index for printing results",
              "starting index for printing results",
			  0,40000000,0,
              G_PARAM_READWRITE);

    properties[PROP_STOP_INDEX]=
        g_param_spec_uint64("stop-index",
			    "ending index for printing results",
			    "ending index for printing results",
			    0,40000000,0,
			    G_PARAM_READWRITE);

    properties[PROP_GET_BOARD_GEN]=
        g_param_spec_uint("get-board-gen",
			  "parameter to define board gen for directgma",
			  "parameter to define board gen for directgma",
			  0,2,0,
			  G_PARAM_READWRITE);
    
    properties[PROP_GET_STREAMING]=
        g_param_spec_uint("get-streaming",
			  "parameter to define streaming for directgma",
			  "parameter to define streaming for directgma",
			  0,2,0,
			  G_PARAM_READWRITE);
    
    properties[PROP_PRINT_PERF]=
        g_param_spec_uint("print-perf",
			  "parameter to print performance or not",
			  "parameter to print performance or not",
			  0,1,0,
			  G_PARAM_READWRITE);

    properties[PROP_GET_AP_SIZE]=
        g_param_spec_uint("get-ap-size",
			  "parameter for auto-detect of ap size or not",
			  "parameter for auto-detect of ap size or not",
			  0,2,0,
			  G_PARAM_READWRITE);

    properties[PROP_ITERATIONS]=
        g_param_spec_uint("iterations",
			  "number of iterations of generate function",
			  "number of iterations of generate function",
			  1,2<<16,1,
			  G_PARAM_READWRITE);

    for (guint i = PROP_0 + 1; i < N_PROPERTIES; i++)
        g_object_class_install_property (oclass, i, properties[i]);

    g_type_class_add_private (oclass, sizeof(UfoDirectGmaTaskPrivate));
}

static void
ufo_direct_gma_task_init(UfoDirectGmaTask *self)
{
    self->priv = UFO_DIRECT_GMA_TASK_GET_PRIVATE(self);
    self->priv->tlp_size=32;
    self->priv->huge_page=1000;
    self->priv->multiple=2;
    self->priv->height=8192;
    self->priv->width=8000;
    self->priv->frames=1;
    self->priv->buffers=8;
    self->priv->counter=0;
    self->priv->stop_index=10;
    self->priv->start_index=0;
    self->priv->get_board_gen=0;
    self->priv->get_streaming=0;
    self->priv->print_perf=0;
    self->priv->print_counter=0;
    self->priv->print_index=0;
    self->priv->get_ap_size=0;
    self->priv->iterations=1;
}
