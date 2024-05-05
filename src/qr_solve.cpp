#define _CRT_SECURE_NO_WARNINGS
#define PROGRAM_FILE "qr_solve.cl"
#define KERNEL_FUNC "qr_solve"

// dimension of matrix
#define MATRIX_DIM 5
// number of matrices for compute once
#define MATRIX_NUM 2
#define MATRIX_TOTAL_ROWS (MATRIX_DIM * MATRIX_NUM)
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

#define __TRACE_LINE printf("func:%s line:%d\n", __func__, __LINE__);

void printMatrix(float matrix[][MATRIX_DIM], int rows, int cols) {
    int i, j;
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            printf("%.4f ", matrix[i][j]);
        }
        printf("\n");
    }
}

void printVector(float *vector, int length) {
    printf("[ ");
    for (int i = 0; i < length; i++) {
        printf("%.4f ", vector[i]);
        if ((i + 1) % 10 == 0)
            printf("\n    ");
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

// 5 columns, 5*n rows.  n is marix number
float a_mat[MATRIX_DIM * MATRIX_NUM][MATRIX_DIM];
float q_mat[MATRIX_DIM * MATRIX_NUM][MATRIX_DIM];
float r_mat[MATRIX_DIM * MATRIX_NUM][MATRIX_DIM];
float check_mat[MATRIX_DIM * MATRIX_NUM][MATRIX_DIM];
// Vector b = -1
float b_vec[MATRIX_DIM * MATRIX_NUM];
// Solution vector x
float x_vec[MATRIX_DIM * MATRIX_NUM];

int main() {

   /* Host/device data structures */
   cl_device_id device;
   cl_context context;
   cl_command_queue queue;
   cl_program program;
   cl_kernel kernel;
   cl_int err, i, j, k, check;

   /* Data and buffers */
   cl_mem a_buffer, q_buffer, p_buffer, prod_buffer, b_buffer, x_buffer;

   // for A matrix init
   float a_buf[MATRIX_DIM * MATRIX_TOTAL_ROWS] = {-2.276, 0.4507, 0.0329, 0, 0, \
                                          -2.273, 0.5457, 0.08334, 0, 0, \
                                          -2.71, 0.2871, -0.05081, 0, 0, \
                                          -2.871, 0.5307, 0.08712, 0, 0, \
                                          -2.337, 0.2689, -0.1338, 0, 0, \
                                          9.775, -0.2636, -0.1972, 0, 0, \
                                          9.235, -0.2776, -0.1902, 0, 0, \
                                          9.746,  0.2444,  -0.193, 0, 0, \
                                          9.26,  0.2579, -0.1672, 0, 0, \
                                          9.731, -0.7539, -0.2024};

   /* Initialize A matrix */
   for(i=0; i<MATRIX_TOTAL_ROWS; i++) {
      for(j=0; j<MATRIX_DIM; j++) {
         a_mat[i][j] = a_buf[i * MATRIX_DIM + j];
         check_mat[i][j] = 0.0f;
      }
   }
   printf("A:\n");
   printMatrix(a_mat, MATRIX_TOTAL_ROWS, MATRIX_DIM);

   /* Initialize B vector */
   for(i=0; i<sizeof(b_vec)/sizeof(b_vec[0]); i++) {
      b_vec[i] = -1.0f;
   }

   // other init, set to zero
   memset(q_mat, 0, sizeof(q_mat));
   memset(r_mat, 0, sizeof(r_mat));
   memset(check_mat, 0, sizeof(check_mat));

   /* Create a device and context */
   device = create_device();
   context = clCreateContext(NULL, 1, &device, NULL, NULL, &err);
   if(err < 0) {
      perror("Couldn't create a context");
      exit(1);
   }

   /* Build the program */
   char cl_path[256];
   sprintf(cl_path, "%ssrc/%s", ROOT_DIR, PROGRAM_FILE);
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
   a_buffer = clCreateBuffer(context,
         CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
         sizeof(a_mat), a_mat, &err);
   if(err < 0) {
      perror("Couldn't create a buffer");
      exit(1);
   };
   q_buffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
         sizeof(q_mat), NULL, NULL);
   p_buffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
         sizeof(q_mat), NULL, NULL);
   prod_buffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
         sizeof(q_mat), NULL, NULL);
   b_buffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(b_vec), b_vec, &err);
   x_buffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(x_vec), NULL, &err);

   /* Create kernel arguments */
   err = clSetKernelArg(kernel, 0, MATRIX_DIM * sizeof(float), NULL);
   err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &a_buffer);
   err |= clSetKernelArg(kernel, 2, sizeof(cl_mem), &q_buffer);
   err |= clSetKernelArg(kernel, 3, sizeof(cl_mem), &p_buffer);
   err |= clSetKernelArg(kernel, 4, sizeof(cl_mem), &prod_buffer);
   err |= clSetKernelArg(kernel, 5, sizeof(cl_mem), &b_buffer);
   err |= clSetKernelArg(kernel, 6, sizeof(cl_mem), &x_buffer);
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
   size_t global_size[2], local_size[2];
   global_size[0] = MATRIX_DIM;
   global_size[1] = MATRIX_NUM; //MATRIX_DIM * MATRIX_NUM;
   local_size[0] = MATRIX_DIM;
   local_size[1] = 1;
   err = clEnqueueNDRangeKernel(queue, kernel, 2, NULL, global_size,
         local_size, 0, NULL, NULL);
   if(err != CL_SUCCESS) {
      printf("Error code: %d\n", err);
      perror("Couldn't enqueue the kernel.");
      exit(1);
   }

   /* Read the results */
   err = clEnqueueReadBuffer(queue, q_buffer, CL_TRUE, 0,
      sizeof(q_mat), q_mat, 0, NULL, NULL);
   err |= clEnqueueReadBuffer(queue, a_buffer, CL_TRUE, 0,
      sizeof(r_mat), r_mat, 0, NULL, NULL);
   err |= clEnqueueReadBuffer(queue, x_buffer, CL_TRUE, 0,
      sizeof(x_vec), x_vec, 0, NULL, NULL);
   if(err < 0) {
      perror("Couldn't read the buffers");
      exit(1);
   }

   auto end      = std::chrono::high_resolution_clock::now();
   auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
   printf("Solve Ax=b takes: %0.3f ms\n", float(duration) / 1e3);

   /* Compute product of Q and R */
   for(i=0; i<MATRIX_DIM; i++) {
      for(j=0; j<MATRIX_DIM; j++) {
         for(k=0; k<MATRIX_DIM; k++) {
            check_mat[i][j] += q_mat[i][k] * r_mat[k][j];
         }
      }
   }

   /* Check data */
   check = 1;
   for(i=0; i<MATRIX_DIM; i++) {
      for(j=0; j<MATRIX_DIM; j++) {
         if(fabs(a_mat[i][j] - check_mat[i][j]) > 0.01f) {
            check = 0;
            break;
         }
      }
   }
   if(check)
      printf("QR decomposition check succeeded.\n");
   else
      printf("QR decomposition check failed.\n");

   printf("A:\n");
   printMatrix(a_mat, MATRIX_TOTAL_ROWS, MATRIX_DIM);
   printf("\nQ:\n");
   printMatrix(q_mat, MATRIX_TOTAL_ROWS, MATRIX_DIM);
   printf("\nR:\n");
   printMatrix(r_mat, MATRIX_TOTAL_ROWS, MATRIX_DIM);
   printf("\ncheck:\n");
   printMatrix(check_mat, MATRIX_TOTAL_ROWS, MATRIX_DIM);
   // x[4] and x[5] have no effect
   for(int n=0; n<MATRIX_NUM; n++){
      x_vec[n*MATRIX_DIM +3]=0.0f;
      x_vec[n*MATRIX_DIM +4]=0.0f;
   }
   printf("\nX:\n");
   printVector(x_vec, 3);
   printVector(x_vec + MATRIX_DIM, 3);

   /* Deallocate resources */
   clReleaseMemObject(a_buffer);
   clReleaseMemObject(q_buffer);
   clReleaseMemObject(p_buffer);
   clReleaseMemObject(prod_buffer);
   clReleaseMemObject(b_buffer);
   clReleaseMemObject(x_buffer);
   clReleaseKernel(kernel);
   clReleaseCommandQueue(queue);
   clReleaseProgram(program);
   clReleaseContext(context);
   return 0;
}
