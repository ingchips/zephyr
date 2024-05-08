/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD);
	while(1) {
		printf("hello\r\n");
		k_sleep(K_MSEC(10000));
	}
	return 0;
}
