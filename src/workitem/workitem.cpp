#define _CRT_SECURE_NO_WARNINGS
#define PROGRAM_FILE "workitem.cl"
#define KERNEL_FUNC "workitem"

#define CL_TARGET_OPENCL_VERSION 120

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <chrono>

#ifdef MAC
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

#include <argp.h>
static char doc[] = "OpenCL workitem test";
static struct argp_option options[] = {
    {"dimension", 'd', "value", 0, "work item dimension"},
    {"item_size0", 0xAA00, "value", 0, "work item size 0"},
    {"item_size1", 0xAA01, "value", 0, "work item size 1"},
    {"item_size2", 0xAA02, "value", 0, "work item size 2"},
    {"group_size", 'g', "value", 0, "work group size"},
    {"verbose", 'v', "level", 0, "debug level"},
    {0}};

struct arguments
{
   int dimension;
   int item_size0;
   int item_size1;
   int item_size2;
   int group_size;
   int verbose;
};

static error_t
parse_opt(int key, char *arg, struct argp_state *state)
{
   struct arguments *arguments = (struct arguments *)state->input;

   switch (key)
   {
   case 'v':
      arguments->verbose = arg ? atoi(arg) : 10;
      break;
   case 'd':
      arguments->dimension = arg ? atoi(arg) : 10;
      break;
   case 'g':
      arguments->group_size = arg ? atoi(arg) : 10;
      break;
   case 0xAA00:
      arguments->item_size0 = arg ? atoi(arg) : 10;
      break;
   case 0xAA01:
      arguments->item_size1 = arg ? atoi(arg) : 10;
      break;
   case 0xAA02:
      arguments->item_size2 = arg ? atoi(arg) : 10;
      break;

   default:
      return ARGP_ERR_UNKNOWN;
   }
   return 0;
}

static struct argp argp = {options, parse_opt, NULL, doc};


#define __TRACE_LINE printf("func:%s line:%d\n", __func__, __LINE__);

void printVector(float *vector, int length) {
    printf("[ ");
    for (int i = 0; i < length; i++) {
        printf("%6.1f ", vector[i]);
        if ((i + 1) % 10 == 0)
            printf("\n  ");
    }
    printf("]\n");
}

/* Find a GPU or CPU associated with the first available platform */
cl_device_id create_device() {

   cl_platform_id platform;
   cl_device_id dev;
   int err;

   /* Identify a platform */
   err = clGetPlatformIDs(1, &platform, NULL);
   if(err < 0) {
      perror("Couldn't identify a platform");
      exit(1);
   }

   /* Access a device */
   err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &dev, NULL);
   if(err == CL_DEVICE_NOT_FOUND) {
      err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_CPU, 1, &dev, NULL);
   }
   if(err < 0) {
      perror("Couldn't access any devices");
      exit(1);
   }

   return dev;
}

/* Create program from a file and compile it */
cl_program build_program(cl_context ctx, cl_device_id dev, const char* filename) {

   cl_program program;
   FILE *program_handle;
   char *program_buffer, *program_log;
   size_t program_size, log_size;
   int err;

   /* Read program file and place content into buffer */
   program_handle = fopen(filename, "r");
   if(program_handle == NULL) {
      perror("Couldn't find the program file");
      exit(1);
   }
   fseek(program_handle, 0, SEEK_END);
   program_size = ftell(program_handle);
   rewind(program_handle);
   program_buffer = (char*)malloc(program_size + 1);
   program_buffer[program_size] = '\0';
   fread(program_buffer, sizeof(char), program_size, program_handle);
   fclose(program_handle);

   /* Create program from file */
   program = clCreateProgramWithSource(ctx, 1,
      (const char**)&program_buffer, &program_size, &err);
   if(err < 0) {
      perror("Couldn't create the program");
      exit(1);
   }
   free(program_buffer);

   /* Build program */
   err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
   if(1) {//err < 0

      /* Find size of log and print to std output */
      clGetProgramBuildInfo(program, dev, CL_PROGRAM_BUILD_LOG,
            0, NULL, &log_size);
      program_log = (char*) malloc(log_size + 1);
      program_log[log_size] = '\0';
      clGetProgramBuildInfo(program, dev, CL_PROGRAM_BUILD_LOG,
            log_size + 1, program_log, NULL);
      printf("%s\n", program_log);
      free(program_log);
      //exit(1);
   }
   return program;
}

int main(int argc, char **argv) {
   /* Host/device data structures */
   cl_device_id device;
   cl_context context;
   cl_command_queue queue;
   cl_program program;
   cl_kernel kernel;
   cl_int err;

   /* Data and buffers */
   cl_mem buffer;

   struct arguments arguments;
   /* Default values. */
   arguments.verbose = 5;
   arguments.dimension = 1;
   arguments.group_size = 5;
   arguments.item_size0 = 1;
   arguments.item_size1 = 0;
   arguments.item_size2 = 0;

   argp_parse(&argp, argc, argv, 0, 0, &arguments);

   printf(" VERBOSE = %d\n DIMENSION = %d\n GROUP_SIZE = %d\n"
          " SIZE0 = %d\n SIZE1 = %d\n SIZE2 = %d\n",
          arguments.verbose, arguments.dimension,
          arguments.group_size,
          arguments.item_size0, arguments.item_size1, arguments.item_size2);
   if(arguments.dimension < 0 || arguments.dimension > 3){
      printf("arguments.dimension=%d invalid!!!!\n", arguments.dimension);
      exit(-1);
   }

   float *vector = NULL;
   int float_num = 1;
   if(arguments.item_size0 > 0)float_num *= arguments.item_size0;
   if(arguments.item_size1 > 0)float_num *= arguments.item_size1;
   if(arguments.item_size2 > 0)float_num *= arguments.item_size2;
   int byte_num = float_num * sizeof(float);
   vector = (float *)malloc(byte_num);
   if(vector == NULL){
      printf("can not malloc %d bytes\n", byte_num);
      exit(-1);
   }else{
      printf("malloc %d bytes for %d float variables succeeded\n", byte_num, float_num);
   }
   for(int i=0; i<float_num; i++){
      vector[i] = 0.0f;
   }

   /* Create a device and context */
   device = create_device();
   context = clCreateContext(NULL, 1, &device, NULL, NULL, &err);
   if(err < 0) {
      perror("Couldn't create a context");
      exit(1);
   }

   /* Build the program */
   char cl_path[256];
   sprintf(cl_path, "%s/%s", ROOT_DIR, PROGRAM_FILE);
   printf("ROOT_DIR:%s\n", ROOT_DIR);
   printf("PROGRAM_FILE:%s\n", PROGRAM_FILE);
   printf("cl_path:%s\n", cl_path);
   program = build_program(context, device, cl_path);

   /* Create a kernel */
   kernel = clCreateKernel(program, KERNEL_FUNC, &err);
   if(err < 0) {
      perror("Couldn't create a kernel");
      exit(1);
   };
   printf("kernel: %p\n", &kernel);

   auto start = std::chrono::high_resolution_clock::now();

   /* Create buffer */
   buffer = clCreateBuffer(context,
         CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
         byte_num, vector, &err);
   if(err < 0) {
      perror("Couldn't create a buffer");
      exit(1);
   };

   /* Create kernel arguments */
   err = clSetKernelArg(kernel, 0, sizeof(cl_mem), &buffer);
   if(err < 0) {
      printf("Couldn't set a kernel argument");
      exit(1);
   };

   /* Create a command queue */
   queue = clCreateCommandQueue(context, device, 0, &err);
   if(err < 0) {
      perror("Couldn't create a command queue");
      exit(1);
   };

   /* Enqueue kernel */
   size_t global_size[3], local_size[3];
   global_size[0] = arguments.item_size0;
   global_size[1] = arguments.item_size1;
   global_size[2] = arguments.item_size2;
   local_size[0] = arguments.item_size0;
   local_size[1] = arguments.item_size1;
   local_size[2] = arguments.item_size2;
   local_size[arguments.dimension -1 ] /= arguments.group_size;
   printf("global_size=[%ld %ld %ld]\n", global_size[0], global_size[1], global_size[2]);
   printf("local_size=[%ld %ld %ld]\n", local_size[0], local_size[1], local_size[2]);
   err = clEnqueueNDRangeKernel(queue, kernel, arguments.dimension, NULL, global_size,
         local_size, 0, NULL, NULL);
   if(err != CL_SUCCESS) {
      printf("Error code: %d\n", err);
      perror("Couldn't enqueue the kernel.");
      exit(1);
   }

   /* Read the results */
   err = clEnqueueReadBuffer(queue, buffer, CL_TRUE, 0,
      byte_num, vector, 0, NULL, NULL);
   if(err < 0) {
      perror("Couldn't read the buffers");
      exit(1);
   }

   auto end      = std::chrono::high_resolution_clock::now();
   auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
   printVector(vector, float_num);
   printf("opencl takes: %0.3f ms\n", float(duration) / 1e3);

   /* Deallocate resources */
   clReleaseMemObject(buffer);
   clReleaseKernel(kernel);
   clReleaseCommandQueue(queue);
   clReleaseProgram(program);
   clReleaseContext(context);
   return 0;
}
