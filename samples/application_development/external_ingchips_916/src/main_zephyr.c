/*  
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* hello world example: calling functions from a static library */


//no use
#include <zephyr.h>
#include <stdio.h>
#include <mylib.h>
#include "main.h"
void main(void)
{
	printf("Hello World!\n");
	// app_main();
	// mylib_hello_world();
}
