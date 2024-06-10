__kernel void workitem(__global float *buffer) {
   int l_id0 = get_local_id(0);
   int g_id0 = get_global_id(0);
   int g_id1 = get_global_id(1);
   int l_s0 = get_local_size(0);
   int g_s0 = get_global_size(0);
   printf("local_id0:%d/%d global_id0:%d/%d g_id1:%d\n", l_id0, l_s0, g_id0, g_s0, g_id1);

   buffer[g_id1*g_s0 + g_id0] = g_id1*g_s0 + g_id0;
}
