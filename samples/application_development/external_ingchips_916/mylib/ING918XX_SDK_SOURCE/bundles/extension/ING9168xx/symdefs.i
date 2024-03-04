--define_symbol att_dispatch_client_can_send_now=0x020053b5
--define_symbol att_dispatch_client_request_can_send_now_event=0x020053bb
--define_symbol att_dispatch_register_client=0x020053c1
--define_symbol att_dispatch_register_server=0x020053d5
--define_symbol att_dispatch_server_can_send_now=0x020053e9
--define_symbol att_dispatch_server_request_can_send_now_event=0x020053ef
--define_symbol att_emit_general_event=0x020054a1
--define_symbol att_server_can_send_packet_now=0x02005bdd
--define_symbol att_server_deferred_read_response=0x02005be1
--define_symbol att_server_get_mtu=0x02005bf9
--define_symbol att_server_indicate=0x02005c71
--define_symbol att_server_init=0x02005cf5
--define_symbol att_server_notify=0x02005d31
--define_symbol att_server_register_packet_handler=0x02005e49
--define_symbol att_server_request_can_send_now_event=0x02005e55
--define_symbol att_set_db=0x02005e71
--define_symbol att_set_read_callback=0x02005e85
--define_symbol att_set_write_callback=0x02005e91
--define_symbol bd_addr_cmp=0x02006061
--define_symbol bd_addr_copy=0x02006067
--define_symbol bd_addr_to_str=0x02006071
--define_symbol big_endian_read_16=0x020060a9
--define_symbol big_endian_read_32=0x020060b1
--define_symbol big_endian_store_16=0x020060c5
--define_symbol big_endian_store_32=0x020060d1
--define_symbol btstack_config=0x02006231
--define_symbol btstack_memory_pool_create=0x0200636f
--define_symbol btstack_memory_pool_free=0x02006399
--define_symbol btstack_memory_pool_get=0x020063f9
--define_symbol btstack_push_user_msg=0x02006461
--define_symbol btstack_push_user_runnable=0x0200646d
--define_symbol btstack_reset=0x02006479
--define_symbol char_for_nibble=0x020067a1
--define_symbol eTaskConfirmSleepModeStatus=0x02006b59
--define_symbol gap_add_dev_to_periodic_list=0x020071c9
--define_symbol gap_add_whitelist=0x020071d9
--define_symbol gap_aes_encrypt=0x020071e5
--define_symbol gap_clear_white_lists=0x0200721d
--define_symbol gap_clr_adv_set=0x02007229
--define_symbol gap_clr_periodic_adv_list=0x02007235
--define_symbol gap_create_connection_cancel=0x02007241
--define_symbol gap_default_periodic_adv_sync_transfer_param=0x0200724d
--define_symbol gap_disconnect=0x02007265
--define_symbol gap_disconnect_all=0x02007291
--define_symbol gap_ext_create_connection=0x020072d1
--define_symbol gap_get_connection_parameter_range=0x020073a9
--define_symbol gap_le_read_channel_map=0x020073e5
--define_symbol gap_periodic_adv_create_sync=0x02007451
--define_symbol gap_periodic_adv_create_sync_cancel=0x02007475
--define_symbol gap_periodic_adv_set_info_transfer=0x02007481
--define_symbol gap_periodic_adv_sync_transfer=0x02007491
--define_symbol gap_periodic_adv_sync_transfer_param=0x020074a1
--define_symbol gap_periodic_adv_term_sync=0x020074bd
--define_symbol gap_read_antenna_info=0x02007545
--define_symbol gap_read_periodic_adv_list_size=0x02007551
--define_symbol gap_read_phy=0x0200755d
--define_symbol gap_read_remote_used_features=0x02007569
--define_symbol gap_read_remote_version=0x02007575
--define_symbol gap_read_rssi=0x02007581
--define_symbol gap_remove_whitelist=0x0200758d
--define_symbol gap_rmv_adv_set=0x02007609
--define_symbol gap_rmv_dev_from_periodic_list=0x02007615
--define_symbol gap_rx_test_v2=0x02007625
--define_symbol gap_rx_test_v3=0x02007635
--define_symbol gap_set_adv_set_random_addr=0x02007681
--define_symbol gap_set_callback_for_next_hci=0x020076bd
--define_symbol gap_set_connection_cte_request_enable=0x020076dd
--define_symbol gap_set_connection_cte_response_enable=0x020076f9
--define_symbol gap_set_connection_cte_rx_param=0x02007709
--define_symbol gap_set_connection_cte_tx_param=0x0200775d
--define_symbol gap_set_connection_parameter_range=0x020077a9
--define_symbol gap_set_connectionless_cte_tx_enable=0x020077c1
--define_symbol gap_set_connectionless_cte_tx_param=0x020077d1
--define_symbol gap_set_connectionless_iq_sampling_enable=0x0200782d
--define_symbol gap_set_data_length=0x02007889
--define_symbol gap_set_def_phy=0x020078a1
--define_symbol gap_set_ext_adv_data=0x020078b1
--define_symbol gap_set_ext_adv_enable=0x020078c9
--define_symbol gap_set_ext_adv_para=0x02007939
--define_symbol gap_set_ext_scan_enable=0x02007a09
--define_symbol gap_set_ext_scan_para=0x02007a21
--define_symbol gap_set_ext_scan_response_data=0x02007ac1
--define_symbol gap_set_host_channel_classification=0x02007ad9
--define_symbol gap_set_periodic_adv_data=0x02007ae9
--define_symbol gap_set_periodic_adv_enable=0x02007b59
--define_symbol gap_set_periodic_adv_para=0x02007b69
--define_symbol gap_set_periodic_adv_rx_enable=0x02007b81
--define_symbol gap_set_phy=0x02007b91
--define_symbol gap_set_random_device_address=0x02007bad
--define_symbol gap_start_ccm=0x02007bdd
--define_symbol gap_test_end=0x02007c25
--define_symbol gap_tx_test_v2=0x02007c31
--define_symbol gap_tx_test_v4=0x02007c49
--define_symbol gap_update_connection_parameters=0x02007c6d
--define_symbol gap_vendor_tx_continuous_wave=0x02007cb1
--define_symbol gatt_client_cancel_write=0x020081d9
--define_symbol gatt_client_discover_characteristic_descriptors=0x020081ff
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x0200823f
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x0200828f
--define_symbol gatt_client_discover_characteristics_for_service=0x020082df
--define_symbol gatt_client_discover_primary_services=0x02008315
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x02008347
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x0200838b
--define_symbol gatt_client_execute_write=0x020083c9
--define_symbol gatt_client_find_included_services_for_service=0x020083ef
--define_symbol gatt_client_get_mtu=0x0200841d
--define_symbol gatt_client_is_ready=0x020084e1
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x020084f7
--define_symbol gatt_client_prepare_write=0x02008519
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x02008555
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x0200857f
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x02008585
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x020085b3
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x020085b9
--define_symbol gatt_client_read_multiple_characteristic_values=0x020085e7
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x02008617
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x02008645
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x02008691
--define_symbol gatt_client_register_handler=0x020086dd
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x020086e9
--define_symbol gatt_client_signed_write_without_response=0x02008b1d
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x02008be1
--define_symbol gatt_client_write_client_characteristic_configuration=0x02008c1b
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x02008c6d
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x02008c7d
--define_symbol gatt_client_write_long_value_of_characteristic=0x02008cb9
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x02008cc9
--define_symbol gatt_client_write_value_of_characteristic=0x02008d05
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x02008d3b
--define_symbol hci_add_event_handler=0x0200a35d
--define_symbol hci_power_control=0x0200ab05
--define_symbol hci_register_acl_packet_handler=0x0200acb9
--define_symbol kv_commit=0x0200b3ed
--define_symbol kv_get=0x0200b449
--define_symbol kv_init=0x0200b455
--define_symbol kv_init_backend=0x0200b4d5
--define_symbol kv_put=0x0200b4e9
--define_symbol kv_remove=0x0200b4f5
--define_symbol kv_remove_all=0x0200b529
--define_symbol kv_value_modified=0x0200b559
--define_symbol kv_value_modified_of_key=0x0200b575
--define_symbol kv_visit=0x0200b581
--define_symbol l2cap_add_event_handler=0x0200b611
--define_symbol l2cap_can_send_packet_now=0x0200b621
--define_symbol l2cap_create_le_credit_based_connection_request=0x0200b7e5
--define_symbol l2cap_credit_based_send=0x0200b92d
--define_symbol l2cap_credit_based_send_continue=0x0200b959
--define_symbol l2cap_disconnect=0x0200b9d5
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0200bbc1
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0200bbdd
--define_symbol l2cap_init=0x0200c01d
--define_symbol l2cap_le_send_flow_control_credit=0x0200c113
--define_symbol l2cap_max_le_mtu=0x0200c421
--define_symbol l2cap_register_packet_handler=0x0200c549
--define_symbol l2cap_register_service=0x0200c555
--define_symbol l2cap_request_can_send_now_event=0x0200c665
--define_symbol l2cap_request_connection_parameter_update=0x0200c67f
--define_symbol l2cap_send_echo_request=0x0200cb61
--define_symbol l2cap_unregister_service=0x0200cc41
--define_symbol le_device_db_add=0x0200cc99
--define_symbol le_device_db_find=0x0200cd71
--define_symbol le_device_db_from_key=0x0200cd9d
--define_symbol le_device_db_iter_cur=0x0200cda5
--define_symbol le_device_db_iter_cur_key=0x0200cda9
--define_symbol le_device_db_iter_init=0x0200cdad
--define_symbol le_device_db_iter_next=0x0200cdb5
--define_symbol le_device_db_remove_key=0x0200cddb
--define_symbol ll_ackable_packet_alloc=0x0200ce07
--define_symbol ll_ackable_packet_get_status=0x0200cf43
--define_symbol ll_ackable_packet_run=0x0200cfb1
--define_symbol ll_ackable_packet_set_tx_data=0x0200d069
--define_symbol ll_adjust_conn_peer_tx_power=0x0200d085
--define_symbol ll_aes_encrypt=0x0200d0b1
--define_symbol ll_allow_nonstandard_adv_type=0x0200d125
--define_symbol ll_attach_cte_to_adv_set=0x0200d13d
--define_symbol ll_channel_monitor_alloc=0x0200d2d5
--define_symbol ll_channel_monitor_check_each_pdu=0x0200d357
--define_symbol ll_channel_monitor_run=0x0200d3b9
--define_symbol ll_config=0x0200d489
--define_symbol ll_free=0x0200d4bf
--define_symbol ll_get_conn_events_info=0x0200d4c9
--define_symbol ll_get_conn_info=0x0200d5b1
--define_symbol ll_get_heap_free_size=0x0200d601
--define_symbol ll_hint_on_ce_len=0x0200d615
--define_symbol ll_legacy_adv_set_interval=0x0200d64d
--define_symbol ll_lock_frequency=0x0200d65d
--define_symbol ll_malloc=0x0200d6c1
--define_symbol ll_override_whitening_init_value=0x0200d6c9
--define_symbol ll_raw_packet_alloc=0x0200d7f1
--define_symbol ll_raw_packet_free=0x0200d8c5
--define_symbol ll_raw_packet_get_bare_rx_data=0x0200d8fd
--define_symbol ll_raw_packet_get_iq_samples=0x0200d9bd
--define_symbol ll_raw_packet_get_rx_data=0x0200da57
--define_symbol ll_raw_packet_recv=0x0200db09
--define_symbol ll_raw_packet_send=0x0200dbdd
--define_symbol ll_raw_packet_set_bare_data=0x0200dcf9
--define_symbol ll_raw_packet_set_bare_mode=0x0200dd37
--define_symbol ll_raw_packet_set_fake_cte_info=0x0200de3b
--define_symbol ll_raw_packet_set_param=0x0200de5d
--define_symbol ll_raw_packet_set_rx_cte=0x0200debf
--define_symbol ll_raw_packet_set_tx_cte=0x0200df55
--define_symbol ll_raw_packet_set_tx_data=0x0200df95
--define_symbol ll_register_hci_acl_previewer=0x0200dffd
--define_symbol ll_scan_set_fixed_channel=0x0200e061
--define_symbol ll_scanner_enable_iq_sampling=0x0200e06d
--define_symbol ll_scanner_enable_iq_sampling_on_legacy=0x0200e115
--define_symbol ll_set_adv_access_address=0x0200e405
--define_symbol ll_set_adv_coded_scheme=0x0200e411
--define_symbol ll_set_conn_acl_report_latency=0x0200e449
--define_symbol ll_set_conn_coded_scheme=0x0200e479
--define_symbol ll_set_conn_interval_unit=0x0200e4a5
--define_symbol ll_set_conn_latency=0x0200e4b1
--define_symbol ll_set_conn_tx_power=0x0200e4e1
--define_symbol ll_set_cte_bit=0x0200e511
--define_symbol ll_set_def_antenna=0x0200e539
--define_symbol ll_set_initiating_coded_scheme=0x0200e559
--define_symbol ll_set_max_conn_number=0x0200e565
--define_symbol ll_set_tx_power_range=0x0200e5f9
--define_symbol ll_unlock_frequency=0x0200e621
--define_symbol nibble_for_char=0x02021525
--define_symbol platform_calibrate_rt_clk=0x020215cf
--define_symbol platform_call_on_stack=0x020030fb
--define_symbol platform_cancel_us_timer=0x020215d3
--define_symbol platform_config=0x020215e9
--define_symbol platform_create_us_timer=0x0202171d
--define_symbol platform_delete_timer=0x02021731
--define_symbol platform_enable_irq=0x02021739
--define_symbol platform_get_current_task=0x0202176d
--define_symbol platform_get_gen_os_driver=0x02021791
--define_symbol platform_get_heap_status=0x02021799
--define_symbol platform_get_link_layer_interf=0x020217b1
--define_symbol platform_get_task_handle=0x020217b9
--define_symbol platform_get_timer_counter=0x020217d9
--define_symbol platform_get_us_time=0x020217dd
--define_symbol platform_get_version=0x020217e1
--define_symbol platform_hrng=0x020217fd
--define_symbol platform_install_isr_stack=0x02021805
--define_symbol platform_install_task_stack=0x02021811
--define_symbol platform_patch_rf_init_data=0x02021849
--define_symbol platform_printf=0x02021855
--define_symbol platform_raise_assertion=0x02021869
--define_symbol platform_rand=0x0202187d
--define_symbol platform_read_info=0x02021881
--define_symbol platform_read_persistent_reg=0x020218b1
--define_symbol platform_reset=0x020218bd
--define_symbol platform_rt_rc_auto_tune=0x020218d1
--define_symbol platform_rt_rc_auto_tune2=0x020218d9
--define_symbol platform_rt_rc_tune=0x02021941
--define_symbol platform_set_abs_timer=0x0202195d
--define_symbol platform_set_evt_callback=0x02021961
--define_symbol platform_set_evt_callback_table=0x02021975
--define_symbol platform_set_irq_callback=0x02021981
--define_symbol platform_set_irq_callback_table=0x0202199d
--define_symbol platform_set_rf_clk_source=0x020219a9
--define_symbol platform_set_rf_init_data=0x020219b5
--define_symbol platform_set_rf_power_mapping=0x020219c1
--define_symbol platform_set_timer=0x020219cd
--define_symbol platform_shutdown=0x020219d1
--define_symbol platform_switch_app=0x020219d5
--define_symbol platform_trace_raw=0x020219ed
--define_symbol platform_write_persistent_reg=0x02021a05
--define_symbol printf_hexdump=0x02021bbd
--define_symbol pvPortMalloc=0x02022695
--define_symbol pvTaskIncrementMutexHeldCount=0x0202277d
--define_symbol pvTimerGetTimerID=0x02022795
--define_symbol pxPortInitialiseStack=0x020227c1
--define_symbol reverse_128=0x020229a9
--define_symbol reverse_24=0x020229af
--define_symbol reverse_256=0x020229b5
--define_symbol reverse_48=0x020229bb
--define_symbol reverse_56=0x020229c1
--define_symbol reverse_64=0x020229c7
--define_symbol reverse_bd_addr=0x020229cd
--define_symbol reverse_bytes=0x020229d3
--define_symbol sm_add_event_handler=0x02022d61
--define_symbol sm_address_resolution_lookup=0x02022eb9
--define_symbol sm_authenticated=0x02023261
--define_symbol sm_authorization_decline=0x0202326f
--define_symbol sm_authorization_grant=0x0202328f
--define_symbol sm_authorization_state=0x020232af
--define_symbol sm_bonding_decline=0x020232c9
--define_symbol sm_config=0x0202371d
--define_symbol sm_config_conn=0x02023751
--define_symbol sm_encryption_key_size=0x0202390b
--define_symbol sm_just_works_confirm=0x02023e91
--define_symbol sm_le_device_key=0x020241d1
--define_symbol sm_passkey_input=0x02024267
--define_symbol sm_private_random_address_generation_get=0x02024621
--define_symbol sm_private_random_address_generation_get_mode=0x02024629
--define_symbol sm_private_random_address_generation_set_mode=0x02024635
--define_symbol sm_private_random_address_generation_set_update_period=0x0202465d
--define_symbol sm_register_external_ltk_callback=0x02024799
--define_symbol sm_register_oob_data_callback=0x020247a5
--define_symbol sm_request_pairing=0x020247b1
--define_symbol sm_send_security_request=0x0202528f
--define_symbol sm_set_accepted_stk_generation_methods=0x020252b5
--define_symbol sm_set_authentication_requirements=0x020252c1
--define_symbol sm_set_encryption_key_size_range=0x020252d1
--define_symbol sscanf_bd_addr=0x02025699
--define_symbol sysSetPublicDeviceAddr=0x02025bd5
--define_symbol uuid128_to_str=0x02026431
--define_symbol uuid_add_bluetooth_prefix=0x02026489
--define_symbol uuid_has_bluetooth_prefix=0x020264a9
--define_symbol uxListRemove=0x020264c5
--define_symbol uxQueueMessagesWaiting=0x020264ed
--define_symbol uxQueueMessagesWaitingFromISR=0x02026515
--define_symbol uxQueueSpacesAvailable=0x02026531
--define_symbol uxTaskGetStackHighWaterMark=0x0202655d
--define_symbol uxTaskPriorityGet=0x0202657d
--define_symbol uxTaskPriorityGetFromISR=0x02026599
--define_symbol vListInitialise=0x02026657
--define_symbol vListInitialiseItem=0x0202666d
--define_symbol vListInsert=0x02026673
--define_symbol vListInsertEnd=0x020266a3
--define_symbol vPortEndScheduler=0x020266bd
--define_symbol vPortEnterCritical=0x020266e9
--define_symbol vPortExitCritical=0x0202672d
--define_symbol vPortFree=0x02026761
--define_symbol vPortSuppressTicksAndSleep=0x02026805
--define_symbol vPortValidateInterruptPriority=0x0202692d
--define_symbol vQueueDelete=0x02026989
--define_symbol vQueueWaitForMessageRestricted=0x020269b5
--define_symbol vTaskDelay=0x020269f9
--define_symbol vTaskInternalSetTimeOutState=0x02026a45
--define_symbol vTaskMissedYield=0x02026a55
--define_symbol vTaskPlaceOnEventList=0x02026a61
--define_symbol vTaskPlaceOnEventListRestricted=0x02026a99
--define_symbol vTaskPriorityDisinheritAfterTimeout=0x02026ad9
--define_symbol vTaskPrioritySet=0x02026b85
--define_symbol vTaskResume=0x02026c4d
--define_symbol vTaskStartScheduler=0x02026cd1
--define_symbol vTaskStepTick=0x02026d61
--define_symbol vTaskSuspend=0x02026d91
--define_symbol vTaskSuspendAll=0x02026e4d
--define_symbol vTaskSwitchContext=0x02026e5d
--define_symbol xPortStartScheduler=0x02026f0d
--define_symbol xQueueAddToSet=0x02027011
--define_symbol xQueueCreateCountingSemaphore=0x02027035
--define_symbol xQueueCreateCountingSemaphoreStatic=0x02027071
--define_symbol xQueueCreateMutex=0x020270b5
--define_symbol xQueueCreateMutexStatic=0x020270cb
--define_symbol xQueueCreateSet=0x020270e5
--define_symbol xQueueGenericCreate=0x020270ed
--define_symbol xQueueGenericCreateStatic=0x02027139
--define_symbol xQueueGenericReset=0x020271a1
--define_symbol xQueueGenericSend=0x0202722d
--define_symbol xQueueGenericSendFromISR=0x02027399
--define_symbol xQueueGiveFromISR=0x02027459
--define_symbol xQueueGiveMutexRecursive=0x020274fd
--define_symbol xQueueIsQueueEmptyFromISR=0x0202753d
--define_symbol xQueueIsQueueFullFromISR=0x02027561
--define_symbol xQueuePeek=0x02027589
--define_symbol xQueuePeekFromISR=0x020276b1
--define_symbol xQueueReceive=0x0202771d
--define_symbol xQueueReceiveFromISR=0x02027849
--define_symbol xQueueRemoveFromSet=0x020278dd
--define_symbol xQueueSelectFromSet=0x020278ff
--define_symbol xQueueSelectFromSetFromISR=0x02027911
--define_symbol xQueueSemaphoreTake=0x02027925
--define_symbol xQueueTakeMutexRecursive=0x02027a91
--define_symbol xTaskCheckForTimeOut=0x02027ad5
--define_symbol xTaskCreate=0x02027b45
--define_symbol xTaskCreateStatic=0x02027ba1
--define_symbol xTaskGetCurrentTaskHandle=0x02027c11
--define_symbol xTaskGetSchedulerState=0x02027c1d
--define_symbol xTaskGetTickCount=0x02027c39
--define_symbol xTaskGetTickCountFromISR=0x02027c45
--define_symbol xTaskIncrementTick=0x02027c55
--define_symbol xTaskPriorityDisinherit=0x02027d21
--define_symbol xTaskPriorityInherit=0x02027db5
--define_symbol xTaskRemoveFromEventList=0x02027e49
--define_symbol xTaskResumeAll=0x02027ec9
--define_symbol xTaskResumeFromISR=0x02027f91
--define_symbol xTimerCreate=0x0202801d
--define_symbol xTimerCreateStatic=0x02028051
--define_symbol xTimerCreateTimerTask=0x02028089
--define_symbol xTimerGenericCommand=0x020280f5
--define_symbol xTimerGetExpiryTime=0x02028165
--define_symbol xTimerGetTimerDaemonTaskHandle=0x02028185
