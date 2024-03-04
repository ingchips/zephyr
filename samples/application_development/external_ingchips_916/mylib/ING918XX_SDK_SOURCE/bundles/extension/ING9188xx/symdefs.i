--define_symbol att_dispatch_client_can_send_now=0x000058bd
--define_symbol att_dispatch_client_request_can_send_now_event=0x000058c3
--define_symbol att_dispatch_register_client=0x000058c9
--define_symbol att_dispatch_register_server=0x000058dd
--define_symbol att_dispatch_server_can_send_now=0x000058f1
--define_symbol att_dispatch_server_request_can_send_now_event=0x000058f7
--define_symbol att_emit_general_event=0x000059a9
--define_symbol att_server_can_send_packet_now=0x000060dd
--define_symbol att_server_deferred_read_response=0x000060e1
--define_symbol att_server_get_mtu=0x000060f9
--define_symbol att_server_indicate=0x00006171
--define_symbol att_server_init=0x000061f5
--define_symbol att_server_notify=0x00006231
--define_symbol att_server_register_packet_handler=0x00006349
--define_symbol att_server_request_can_send_now_event=0x00006355
--define_symbol att_set_db=0x00006371
--define_symbol att_set_read_callback=0x00006385
--define_symbol att_set_write_callback=0x00006391
--define_symbol bd_addr_cmp=0x00006501
--define_symbol bd_addr_copy=0x00006507
--define_symbol bd_addr_to_str=0x00006511
--define_symbol big_endian_read_16=0x00006549
--define_symbol big_endian_read_32=0x00006551
--define_symbol big_endian_store_16=0x00006565
--define_symbol big_endian_store_32=0x00006571
--define_symbol btstack_config=0x000066c5
--define_symbol btstack_memory_pool_create=0x00006803
--define_symbol btstack_memory_pool_free=0x0000682d
--define_symbol btstack_memory_pool_get=0x0000688d
--define_symbol btstack_push_user_msg=0x000068f5
--define_symbol btstack_push_user_runnable=0x00006901
--define_symbol btstack_reset=0x0000690d
--define_symbol char_for_nibble=0x00006be9
--define_symbol eTaskConfirmSleepModeStatus=0x00006ed1
--define_symbol gap_add_dev_to_periodic_list=0x0000753d
--define_symbol gap_add_whitelist=0x0000754d
--define_symbol gap_aes_encrypt=0x00007559
--define_symbol gap_clear_white_lists=0x00007591
--define_symbol gap_clr_adv_set=0x0000759d
--define_symbol gap_clr_periodic_adv_list=0x000075a9
--define_symbol gap_create_connection_cancel=0x000075b5
--define_symbol gap_default_periodic_adv_sync_transfer_param=0x000075c1
--define_symbol gap_disconnect=0x000075d9
--define_symbol gap_disconnect_all=0x00007605
--define_symbol gap_ext_create_connection=0x00007645
--define_symbol gap_get_connection_parameter_range=0x0000771d
--define_symbol gap_le_read_channel_map=0x00007759
--define_symbol gap_periodic_adv_create_sync=0x000077c5
--define_symbol gap_periodic_adv_create_sync_cancel=0x000077e9
--define_symbol gap_periodic_adv_set_info_transfer=0x000077f5
--define_symbol gap_periodic_adv_sync_transfer=0x00007805
--define_symbol gap_periodic_adv_sync_transfer_param=0x00007815
--define_symbol gap_periodic_adv_term_sync=0x00007831
--define_symbol gap_read_antenna_info=0x000078b9
--define_symbol gap_read_periodic_adv_list_size=0x000078c5
--define_symbol gap_read_phy=0x000078d1
--define_symbol gap_read_remote_used_features=0x000078dd
--define_symbol gap_read_remote_version=0x000078e9
--define_symbol gap_read_rssi=0x000078f5
--define_symbol gap_remove_whitelist=0x00007901
--define_symbol gap_rmv_adv_set=0x0000797d
--define_symbol gap_rmv_dev_from_periodic_list=0x00007989
--define_symbol gap_rx_test_v2=0x00007999
--define_symbol gap_rx_test_v3=0x000079a9
--define_symbol gap_set_adv_set_random_addr=0x000079f5
--define_symbol gap_set_callback_for_next_hci=0x00007a31
--define_symbol gap_set_connection_cte_request_enable=0x00007a51
--define_symbol gap_set_connection_cte_response_enable=0x00007a6d
--define_symbol gap_set_connection_cte_rx_param=0x00007a7d
--define_symbol gap_set_connection_cte_tx_param=0x00007ad1
--define_symbol gap_set_connection_parameter_range=0x00007b1d
--define_symbol gap_set_connectionless_cte_tx_enable=0x00007b35
--define_symbol gap_set_connectionless_cte_tx_param=0x00007b45
--define_symbol gap_set_connectionless_iq_sampling_enable=0x00007ba1
--define_symbol gap_set_data_length=0x00007bfd
--define_symbol gap_set_def_phy=0x00007c15
--define_symbol gap_set_ext_adv_data=0x00007c25
--define_symbol gap_set_ext_adv_enable=0x00007c3d
--define_symbol gap_set_ext_adv_para=0x00007cad
--define_symbol gap_set_ext_scan_enable=0x00007d85
--define_symbol gap_set_ext_scan_para=0x00007d9d
--define_symbol gap_set_ext_scan_response_data=0x00007e3d
--define_symbol gap_set_host_channel_classification=0x00007e55
--define_symbol gap_set_periodic_adv_data=0x00007e65
--define_symbol gap_set_periodic_adv_enable=0x00007ed5
--define_symbol gap_set_periodic_adv_para=0x00007ee5
--define_symbol gap_set_periodic_adv_rx_enable=0x00007efd
--define_symbol gap_set_phy=0x00007f0d
--define_symbol gap_set_random_device_address=0x00007f29
--define_symbol gap_start_ccm=0x00007f59
--define_symbol gap_test_end=0x00007fa1
--define_symbol gap_tx_test_v2=0x00007fad
--define_symbol gap_tx_test_v4=0x00007fc5
--define_symbol gap_update_connection_parameters=0x00007fe9
--define_symbol gap_vendor_tx_continuous_wave=0x0000802d
--define_symbol gatt_client_cancel_write=0x00008555
--define_symbol gatt_client_discover_characteristic_descriptors=0x0000857b
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x000085bb
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x0000860b
--define_symbol gatt_client_discover_characteristics_for_service=0x0000865b
--define_symbol gatt_client_discover_primary_services=0x00008691
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x000086c3
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x00008707
--define_symbol gatt_client_execute_write=0x00008743
--define_symbol gatt_client_find_included_services_for_service=0x00008769
--define_symbol gatt_client_get_mtu=0x00008797
--define_symbol gatt_client_is_ready=0x00008839
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x0000884f
--define_symbol gatt_client_prepare_write=0x00008871
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x000088ad
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x000088d7
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x000088dd
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x0000890b
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x00008911
--define_symbol gatt_client_read_multiple_characteristic_values=0x0000893f
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x0000896f
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x0000899d
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x000089e9
--define_symbol gatt_client_register_handler=0x00008a35
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x00008a41
--define_symbol gatt_client_signed_write_without_response=0x00008e71
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008f35
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008f6f
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008fc1
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008fd1
--define_symbol gatt_client_write_long_value_of_characteristic=0x0000900d
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x0000901d
--define_symbol gatt_client_write_value_of_characteristic=0x00009059
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x0000908f
--define_symbol hci_add_event_handler=0x0000a5d1
--define_symbol hci_power_control=0x0000ad71
--define_symbol hci_register_acl_packet_handler=0x0000af25
--define_symbol kv_commit=0x0000b699
--define_symbol kv_get=0x0000b6f5
--define_symbol kv_init=0x0000b701
--define_symbol kv_init_backend=0x0000b781
--define_symbol kv_put=0x0000b795
--define_symbol kv_remove=0x0000b7a1
--define_symbol kv_remove_all=0x0000b7d5
--define_symbol kv_value_modified=0x0000b805
--define_symbol kv_value_modified_of_key=0x0000b821
--define_symbol kv_visit=0x0000b82d
--define_symbol l2cap_add_event_handler=0x0000b8bd
--define_symbol l2cap_can_send_packet_now=0x0000b8cd
--define_symbol l2cap_create_le_credit_based_connection_request=0x0000ba89
--define_symbol l2cap_credit_based_send=0x0000bbcd
--define_symbol l2cap_credit_based_send_continue=0x0000bbf9
--define_symbol l2cap_disconnect=0x0000bc75
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0000bec5
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0000bee1
--define_symbol l2cap_init=0x0000c2b5
--define_symbol l2cap_le_send_flow_control_credit=0x0000c3ab
--define_symbol l2cap_max_le_mtu=0x0000c6b5
--define_symbol l2cap_register_packet_handler=0x0000c7dd
--define_symbol l2cap_register_service=0x0000c7e9
--define_symbol l2cap_request_can_send_now_event=0x0000c8f9
--define_symbol l2cap_request_connection_parameter_update=0x0000c913
--define_symbol l2cap_send_echo_request=0x0000cded
--define_symbol l2cap_unregister_service=0x0000cead
--define_symbol le_device_db_add=0x0000cf05
--define_symbol le_device_db_find=0x0000cfdd
--define_symbol le_device_db_from_key=0x0000d009
--define_symbol le_device_db_iter_cur=0x0000d011
--define_symbol le_device_db_iter_cur_key=0x0000d015
--define_symbol le_device_db_iter_init=0x0000d019
--define_symbol le_device_db_iter_next=0x0000d021
--define_symbol le_device_db_remove_key=0x0000d047
--define_symbol ll_ackable_packet_alloc=0x0000d073
--define_symbol ll_ackable_packet_get_status=0x0000d1a5
--define_symbol ll_ackable_packet_run=0x0000d215
--define_symbol ll_ackable_packet_set_tx_data=0x0000d2b1
--define_symbol ll_aes_encrypt=0x0000d2cd
--define_symbol ll_attach_cte_to_adv_set=0x0000d349
--define_symbol ll_channel_monitor_alloc=0x0000d4e1
--define_symbol ll_channel_monitor_check_each_pdu=0x0000d563
--define_symbol ll_channel_monitor_run=0x0000d5c9
--define_symbol ll_config=0x0000d67d
--define_symbol ll_free=0x0000d6b3
--define_symbol ll_get_heap_free_size=0x0000d6bd
--define_symbol ll_hint_on_ce_len=0x0000d6d1
--define_symbol ll_legacy_adv_set_interval=0x0000d709
--define_symbol ll_lock_frequency=0x0000d719
--define_symbol ll_malloc=0x0000d77d
--define_symbol ll_query_timing_info=0x0000d8b5
--define_symbol ll_raw_packet_alloc=0x0000d901
--define_symbol ll_raw_packet_free=0x0000d9d5
--define_symbol ll_raw_packet_get_bare_rx_data=0x0000da0d
--define_symbol ll_raw_packet_get_iq_samples=0x0000dad3
--define_symbol ll_raw_packet_get_rx_data=0x0000db6d
--define_symbol ll_raw_packet_recv=0x0000dc0d
--define_symbol ll_raw_packet_send=0x0000dcc9
--define_symbol ll_raw_packet_set_bare_data=0x0000ddb1
--define_symbol ll_raw_packet_set_bare_mode=0x0000ddef
--define_symbol ll_raw_packet_set_fake_cte_info=0x0000def5
--define_symbol ll_raw_packet_set_param=0x0000df17
--define_symbol ll_raw_packet_set_rx_cte=0x0000df75
--define_symbol ll_raw_packet_set_tx_cte=0x0000e00b
--define_symbol ll_raw_packet_set_tx_data=0x0000e049
--define_symbol ll_register_hci_acl_previewer=0x0000e0ad
--define_symbol ll_scan_set_fixed_channel=0x0000e111
--define_symbol ll_scanner_enable_iq_sampling=0x0000e11d
--define_symbol ll_set_adv_access_address=0x0000e3d1
--define_symbol ll_set_adv_coded_scheme=0x0000e3dd
--define_symbol ll_set_conn_acl_report_latency=0x0000e40d
--define_symbol ll_set_conn_coded_scheme=0x0000e43d
--define_symbol ll_set_conn_interval_unit=0x0000e469
--define_symbol ll_set_conn_latency=0x0000e475
--define_symbol ll_set_conn_tx_power=0x0000e4a5
--define_symbol ll_set_def_antenna=0x0000e4ed
--define_symbol ll_set_initiating_coded_scheme=0x0000e509
--define_symbol ll_set_max_conn_number=0x0000e515
--define_symbol ll_unlock_frequency=0x0000e5a9
--define_symbol nibble_for_char=0x0001f869
--define_symbol platform_calibrate_rt_clk=0x0001f915
--define_symbol platform_call_on_stack=0x00004183
--define_symbol platform_cancel_us_timer=0x0001f919
--define_symbol platform_config=0x0001f92d
--define_symbol platform_create_us_timer=0x0001fa51
--define_symbol platform_delete_timer=0x0001fa65
--define_symbol platform_enable_irq=0x0001fa6d
--define_symbol platform_get_current_task=0x0001faa5
--define_symbol platform_get_gen_os_driver=0x0001fac9
--define_symbol platform_get_heap_status=0x0001fad1
--define_symbol platform_get_link_layer_interf=0x0001fae9
--define_symbol platform_get_task_handle=0x0001faf1
--define_symbol platform_get_timer_counter=0x0001fb11
--define_symbol platform_get_us_time=0x0001fb15
--define_symbol platform_get_version=0x0001fb19
--define_symbol platform_hrng=0x0001fb21
--define_symbol platform_install_isr_stack=0x0001fb29
--define_symbol platform_install_task_stack=0x0001fb35
--define_symbol platform_patch_rf_init_data=0x0001fb6d
--define_symbol platform_printf=0x0001fb79
--define_symbol platform_raise_assertion=0x0001fb8d
--define_symbol platform_rand=0x0001fba1
--define_symbol platform_read_info=0x0001fba5
--define_symbol platform_read_persistent_reg=0x0001fbd5
--define_symbol platform_reset=0x0001fbe5
--define_symbol platform_rt_rc_auto_tune=0x0001fc09
--define_symbol platform_rt_rc_auto_tune2=0x0001fc11
--define_symbol platform_rt_rc_tune=0x0001fc99
--define_symbol platform_set_abs_timer=0x0001fcad
--define_symbol platform_set_evt_callback=0x0001fcb1
--define_symbol platform_set_evt_callback_table=0x0001fcc5
--define_symbol platform_set_irq_callback=0x0001fcd1
--define_symbol platform_set_irq_callback_table=0x0001fced
--define_symbol platform_set_rf_clk_source=0x0001fcf9
--define_symbol platform_set_rf_init_data=0x0001fd05
--define_symbol platform_set_rf_power_mapping=0x0001fd11
--define_symbol platform_set_timer=0x0001fd1d
--define_symbol platform_shutdown=0x0001fd21
--define_symbol platform_switch_app=0x0001fd25
--define_symbol platform_trace_raw=0x0001fd51
--define_symbol platform_write_persistent_reg=0x0001fd69
--define_symbol printf_hexdump=0x0001ff1d
--define_symbol pvPortMalloc=0x00020a11
--define_symbol pvTaskIncrementMutexHeldCount=0x00020af9
--define_symbol pvTimerGetTimerID=0x00020b11
--define_symbol pxPortInitialiseStack=0x00020b3d
--define_symbol reverse_128=0x00020d1d
--define_symbol reverse_24=0x00020d23
--define_symbol reverse_256=0x00020d29
--define_symbol reverse_48=0x00020d2f
--define_symbol reverse_56=0x00020d35
--define_symbol reverse_64=0x00020d3b
--define_symbol reverse_bd_addr=0x00020d41
--define_symbol reverse_bytes=0x00020d47
--define_symbol sm_add_event_handler=0x00021065
--define_symbol sm_address_resolution_lookup=0x000211bd
--define_symbol sm_authenticated=0x00021565
--define_symbol sm_authorization_decline=0x00021573
--define_symbol sm_authorization_grant=0x00021593
--define_symbol sm_authorization_state=0x000215b3
--define_symbol sm_bonding_decline=0x000215cd
--define_symbol sm_config=0x00021a29
--define_symbol sm_config_conn=0x00021a5d
--define_symbol sm_encryption_key_size=0x00021c17
--define_symbol sm_just_works_confirm=0x0002219d
--define_symbol sm_le_device_key=0x000224dd
--define_symbol sm_passkey_input=0x00022573
--define_symbol sm_private_random_address_generation_get=0x0002292d
--define_symbol sm_private_random_address_generation_get_mode=0x00022935
--define_symbol sm_private_random_address_generation_set_mode=0x00022941
--define_symbol sm_private_random_address_generation_set_update_period=0x00022969
--define_symbol sm_register_external_ltk_callback=0x00022aa5
--define_symbol sm_register_oob_data_callback=0x00022ab1
--define_symbol sm_request_pairing=0x00022abd
--define_symbol sm_send_security_request=0x0002359b
--define_symbol sm_set_accepted_stk_generation_methods=0x000235c1
--define_symbol sm_set_authentication_requirements=0x000235cd
--define_symbol sm_set_encryption_key_size_range=0x000235dd
--define_symbol sscanf_bd_addr=0x0002399d
--define_symbol sysSetPublicDeviceAddr=0x00023d51
--define_symbol uuid128_to_str=0x000244fd
--define_symbol uuid_add_bluetooth_prefix=0x00024555
--define_symbol uuid_has_bluetooth_prefix=0x00024575
--define_symbol uxListRemove=0x00024591
--define_symbol uxQueueMessagesWaiting=0x000245b9
--define_symbol uxQueueMessagesWaitingFromISR=0x000245e1
--define_symbol uxQueueSpacesAvailable=0x000245fd
--define_symbol uxTaskGetStackHighWaterMark=0x00024629
--define_symbol uxTaskPriorityGet=0x00024649
--define_symbol uxTaskPriorityGetFromISR=0x00024665
--define_symbol vListInitialise=0x0002471f
--define_symbol vListInitialiseItem=0x00024735
--define_symbol vListInsert=0x0002473b
--define_symbol vListInsertEnd=0x0002476b
--define_symbol vPortEndScheduler=0x00024785
--define_symbol vPortEnterCritical=0x000247b1
--define_symbol vPortExitCritical=0x000247f5
--define_symbol vPortFree=0x00024829
--define_symbol vPortSuppressTicksAndSleep=0x000248bd
--define_symbol vPortValidateInterruptPriority=0x000249e5
--define_symbol vQueueDelete=0x00024a41
--define_symbol vQueueWaitForMessageRestricted=0x00024a6d
--define_symbol vTaskDelay=0x00024ab5
--define_symbol vTaskInternalSetTimeOutState=0x00024b01
--define_symbol vTaskMissedYield=0x00024b11
--define_symbol vTaskPlaceOnEventList=0x00024b1d
--define_symbol vTaskPlaceOnEventListRestricted=0x00024b55
--define_symbol vTaskPriorityDisinheritAfterTimeout=0x00024b95
--define_symbol vTaskPrioritySet=0x00024c41
--define_symbol vTaskResume=0x00024d09
--define_symbol vTaskStartScheduler=0x00024d8d
--define_symbol vTaskStepTick=0x00024e1d
--define_symbol vTaskSuspend=0x00024e4d
--define_symbol vTaskSuspendAll=0x00024f09
--define_symbol vTaskSwitchContext=0x00024f19
--define_symbol xPortStartScheduler=0x00024fc9
--define_symbol xQueueAddToSet=0x00025091
--define_symbol xQueueCreateCountingSemaphore=0x000250b5
--define_symbol xQueueCreateCountingSemaphoreStatic=0x000250f1
--define_symbol xQueueCreateMutex=0x00025135
--define_symbol xQueueCreateMutexStatic=0x0002514b
--define_symbol xQueueCreateSet=0x00025165
--define_symbol xQueueGenericCreate=0x0002516d
--define_symbol xQueueGenericCreateStatic=0x000251b9
--define_symbol xQueueGenericReset=0x00025221
--define_symbol xQueueGenericSend=0x000252ad
--define_symbol xQueueGenericSendFromISR=0x00025419
--define_symbol xQueueGiveFromISR=0x000254d9
--define_symbol xQueueGiveMutexRecursive=0x0002557d
--define_symbol xQueueIsQueueEmptyFromISR=0x000255bd
--define_symbol xQueueIsQueueFullFromISR=0x000255e1
--define_symbol xQueuePeek=0x00025609
--define_symbol xQueuePeekFromISR=0x00025731
--define_symbol xQueueReceive=0x0002579d
--define_symbol xQueueReceiveFromISR=0x000258c9
--define_symbol xQueueRemoveFromSet=0x0002595d
--define_symbol xQueueSelectFromSet=0x0002597f
--define_symbol xQueueSelectFromSetFromISR=0x00025991
--define_symbol xQueueSemaphoreTake=0x000259a5
--define_symbol xQueueTakeMutexRecursive=0x00025b11
--define_symbol xTaskCheckForTimeOut=0x00025b55
--define_symbol xTaskCreate=0x00025bc5
--define_symbol xTaskCreateStatic=0x00025c21
--define_symbol xTaskGetCurrentTaskHandle=0x00025c91
--define_symbol xTaskGetSchedulerState=0x00025c9d
--define_symbol xTaskGetTickCount=0x00025cb9
--define_symbol xTaskGetTickCountFromISR=0x00025cc5
--define_symbol xTaskIncrementTick=0x00025cd5
--define_symbol xTaskPriorityDisinherit=0x00025da1
--define_symbol xTaskPriorityInherit=0x00025e35
--define_symbol xTaskRemoveFromEventList=0x00025ec9
--define_symbol xTaskResumeAll=0x00025f49
--define_symbol xTaskResumeFromISR=0x00026011
--define_symbol xTimerCreate=0x0002609d
--define_symbol xTimerCreateStatic=0x000260d1
--define_symbol xTimerCreateTimerTask=0x00026109
--define_symbol xTimerGenericCommand=0x00026175
--define_symbol xTimerGetExpiryTime=0x000261e5
--define_symbol xTimerGetTimerDaemonTaskHandle=0x00026205
