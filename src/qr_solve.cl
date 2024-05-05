__kernel void qr_solve(__local float *u_vec, __global float *a_mat, 
                       __global float *q_mat, __global float *p_mat, 
                       __global float *prod_mat, __global const float *b, 
                       __global float *x) {


   local float u_length_squared, dot;
   float prod, vec_length = 0.0f;

   int id = get_local_id(0);
   int num_cols = get_global_size(0);

   /* Load first column into local memory as u vector */
   u_vec[id] = a_mat[id*num_cols];
   barrier(CLK_LOCAL_MEM_FENCE);

   /* Find length of first A column and u vector */
   if(id == 0) {
      for(int i=1; i<num_cols; i++) {
         vec_length += u_vec[i] * u_vec[i];
      }
      u_length_squared = vec_length;
      if (u_length_squared == 0.0f) {
          // Handle the case of zero columns in a_mat
          // Set u_length_squared to a small value to avoid division by zero
          u_length_squared = 1.0e-10f;
      }
      vec_length = sqrt(vec_length + u_vec[0] * u_vec[0]);
      a_mat[0] = vec_length;
      u_vec[0] -= vec_length;
      u_length_squared += u_vec[0] * u_vec[0];
   }
   else {
      a_mat[id*num_cols] = 0.0f;
   }
   barrier(CLK_GLOBAL_MEM_FENCE);

   /* Transform further columns of A */
   for(int i=1; i<num_cols; i++) {
      dot = 0.0f;
      if(id == 0) {
         for(int j=0; j<num_cols; j++) {
            dot += a_mat[j*num_cols + i] * u_vec[j];
         }
      }
      barrier(CLK_LOCAL_MEM_FENCE);
      a_mat[id*num_cols + i] -= 2 * u_vec[id] * dot / u_length_squared;
   }

   /* Update Q matrix */
   for(int i=0; i<num_cols; i++) {
      q_mat[id*num_cols + i] = -2 * u_vec[i] * 
            u_vec[id] / u_length_squared;
   }
   q_mat[id*num_cols + id] += 1;
   barrier(CLK_GLOBAL_MEM_FENCE); 

   /* Loop through other columns */
   for(int col = 1; col < num_cols-1; col++) {

      /* Load new column into memory */
      u_vec[id] = a_mat[id * num_cols + col];
      barrier(CLK_LOCAL_MEM_FENCE);

      /* Find length of A column and u vector */
      if(id == col) {
         vec_length = 0.0f;
         for(int i = col + 1; i < num_cols; i++) {
            vec_length += u_vec[i] * u_vec[i];
         }
         u_length_squared = vec_length;
         if (u_length_squared == 0.0f) {
             // Handle the case of zero columns in a_mat
             // Set u_length_squared to a small value to avoid division by zero
             u_length_squared = 1.0e-10f;
         }
         vec_length = sqrt(vec_length + u_vec[col] * u_vec[col]);
         u_vec[col] -= vec_length;
         u_length_squared += u_vec[col] * u_vec[col];
         a_mat[col * num_cols + col] = vec_length;
      }
      else if(id > col) {
         a_mat[id * num_cols + col] = 0.0f;
      }
      barrier(CLK_GLOBAL_MEM_FENCE);

      /* Transform further columns of A */
      for(int i = col+1; i < num_cols; i++) {
         if(id == 0) {
            dot = 0.0f;
            for(int j=col; j<num_cols; j++) {
               dot += a_mat[j*num_cols + i] * u_vec[j];
            }
         }
         barrier(CLK_LOCAL_MEM_FENCE);
         
         if(id >= col)
            a_mat[id*num_cols + i] -= 2 * u_vec[id] * 
                  dot / u_length_squared;
         barrier(CLK_GLOBAL_MEM_FENCE);
      }

      /* Update P matrix */
      if(id >= col) {
         for(int i=col; i<num_cols; i++) {
            p_mat[id*num_cols + i] = -2 * u_vec[i] * 
                  u_vec[id] / u_length_squared;
         }
         p_mat[id*num_cols + id] += 1;
      }
      barrier(CLK_GLOBAL_MEM_FENCE); 

      /* Multiply q_mat * p_mat = prod_mat */
      for(int i=col; i<num_cols; i++) {
         prod = 0.0f;
         for(int j=col; j<num_cols; j++) {
            prod += q_mat[id*num_cols + j] * p_mat[j*num_cols + i];
         }     
         prod_mat[id*num_cols + i] = prod;  
      }
      barrier(CLK_GLOBAL_MEM_FENCE); 

      /* Place the content of prod_mat in q_mat */
      for(int i=col; i<num_cols; i++) {
         q_mat[id*num_cols + i] = prod_mat[id*num_cols + i];
      }
      barrier(CLK_GLOBAL_MEM_FENCE); 
   }

    /* Compute Q^Tb */
    float y_val = 0.0f;
    if (id < num_cols) {
        for (int i = 0; i < num_cols; i++) {
            y_val += q_mat[i * num_cols + id] * b[i]; // Q is accessed by transposing on-the-fly
        }
    }
    barrier(CLK_GLOBAL_MEM_FENCE);

    /* Solve Rx = y using back-substitution */
    float sum = y_val;
    for (int i = num_cols - 1; i >= id; i--) {
        barrier(CLK_LOCAL_MEM_FENCE);
        if (id == i) {
            for (int j = i + 1; j < num_cols; j++) {
                sum -= a_mat[i * num_cols + j] * x[j]; // Use updated a_mat as R
            }
            if (a_mat[i * num_cols + i] == 0.0f) {
               // Set to a small value to avoid division by zero
               a_mat[i * num_cols + i] = 1.0e-10f;
            }
            x[i] = sum / a_mat[i * num_cols + i];
            sum = x[i]; // Prepare sum for the next iteration if needed
        }
    }
    barrier(CLK_GLOBAL_MEM_FENCE);
}
