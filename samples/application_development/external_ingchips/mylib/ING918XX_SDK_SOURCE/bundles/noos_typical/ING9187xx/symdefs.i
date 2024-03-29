--define_symbol att_dispatch_client_can_send_now=0x000056a5
--define_symbol att_dispatch_client_request_can_send_now_event=0x000056ab
--define_symbol att_dispatch_register_client=0x000056b1
--define_symbol att_dispatch_register_server=0x000056c5
--define_symbol att_dispatch_server_can_send_now=0x000056d9
--define_symbol att_dispatch_server_request_can_send_now_event=0x000056df
--define_symbol att_emit_general_event=0x00005791
--define_symbol att_server_can_send_packet_now=0x00005ec5
--define_symbol att_server_deferred_read_response=0x00005ec9
--define_symbol att_server_get_mtu=0x00005ee1
--define_symbol att_server_indicate=0x00005f59
--define_symbol att_server_init=0x00005fdd
--define_symbol att_server_notify=0x00006019
--define_symbol att_server_register_packet_handler=0x00006131
--define_symbol att_server_request_can_send_now_event=0x0000613d
--define_symbol att_set_db=0x00006159
--define_symbol att_set_read_callback=0x0000616d
--define_symbol att_set_write_callback=0x00006179
--define_symbol bd_addr_cmp=0x000062e9
--define_symbol bd_addr_copy=0x000062ef
--define_symbol bd_addr_to_str=0x000062f9
--define_symbol big_endian_read_16=0x00006331
--define_symbol big_endian_read_32=0x00006339
--define_symbol big_endian_store_16=0x0000634d
--define_symbol big_endian_store_32=0x00006359
--define_symbol btstack_config=0x00006491
--define_symbol btstack_memory_pool_create=0x000065df
--define_symbol btstack_memory_pool_free=0x00006609
--define_symbol btstack_memory_pool_get=0x00006669
--define_symbol btstack_push_user_msg=0x000066b1
--define_symbol btstack_push_user_runnable=0x000066bd
--define_symbol btstack_reset=0x000066c9
--define_symbol char_for_nibble=0x0000698d
--define_symbol gap_add_dev_to_periodic_list=0x00007231
--define_symbol gap_add_whitelist=0x00007241
--define_symbol gap_aes_encrypt=0x0000724d
--define_symbol gap_clear_white_lists=0x00007285
--define_symbol gap_clr_adv_set=0x00007291
--define_symbol gap_clr_periodic_adv_list=0x0000729d
--define_symbol gap_create_connection_cancel=0x000072a9
--define_symbol gap_disconnect=0x000072b5
--define_symbol gap_disconnect_all=0x000072e1
--define_symbol gap_ext_create_connection=0x00007321
--define_symbol gap_get_connection_parameter_range=0x000073f9
--define_symbol gap_le_read_channel_map=0x00007435
--define_symbol gap_periodic_adv_create_sync=0x000074a1
--define_symbol gap_periodic_adv_create_sync_cancel=0x000074c5
--define_symbol gap_periodic_adv_term_sync=0x000074d1
--define_symbol gap_read_periodic_adv_list_size=0x00007559
--define_symbol gap_read_phy=0x00007565
--define_symbol gap_read_remote_used_features=0x00007571
--define_symbol gap_read_remote_version=0x0000757d
--define_symbol gap_read_rssi=0x00007589
--define_symbol gap_remove_whitelist=0x00007595
--define_symbol gap_rmv_adv_set=0x00007611
--define_symbol gap_rmv_dev_from_periodic_list=0x0000761d
--define_symbol gap_rx_test_v2=0x0000762d
--define_symbol gap_set_adv_set_random_addr=0x00007665
--define_symbol gap_set_callback_for_next_hci=0x000076a1
--define_symbol gap_set_connection_parameter_range=0x000076c1
--define_symbol gap_set_data_length=0x000076d9
--define_symbol gap_set_def_phy=0x000076f1
--define_symbol gap_set_ext_adv_data=0x00007701
--define_symbol gap_set_ext_adv_enable=0x00007719
--define_symbol gap_set_ext_adv_para=0x00007789
--define_symbol gap_set_ext_scan_enable=0x00007861
--define_symbol gap_set_ext_scan_para=0x00007879
--define_symbol gap_set_ext_scan_response_data=0x00007919
--define_symbol gap_set_host_channel_classification=0x00007931
--define_symbol gap_set_periodic_adv_data=0x00007941
--define_symbol gap_set_periodic_adv_enable=0x000079b1
--define_symbol gap_set_periodic_adv_para=0x000079c1
--define_symbol gap_set_phy=0x000079d9
--define_symbol gap_set_random_device_address=0x000079f5
--define_symbol gap_start_ccm=0x00007a25
--define_symbol gap_test_end=0x00007a6d
--define_symbol gap_tx_test_v2=0x00007a79
--define_symbol gap_tx_test_v4=0x00007a91
--define_symbol gap_update_connection_parameters=0x00007ab5
--define_symbol gap_vendor_tx_continuous_wave=0x00007af9
--define_symbol gatt_client_cancel_write=0x00008021
--define_symbol gatt_client_discover_characteristic_descriptors=0x00008047
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x00008087
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x000080d7
--define_symbol gatt_client_discover_characteristics_for_service=0x00008127
--define_symbol gatt_client_discover_primary_services=0x0000815d
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x0000818f
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x000081d3
--define_symbol gatt_client_execute_write=0x0000820f
--define_symbol gatt_client_find_included_services_for_service=0x00008235
--define_symbol gatt_client_get_mtu=0x00008263
--define_symbol gatt_client_is_ready=0x00008305
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x0000831b
--define_symbol gatt_client_prepare_write=0x0000833d
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x00008379
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x000083a3
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x000083a9
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x000083d7
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x000083dd
--define_symbol gatt_client_read_multiple_characteristic_values=0x0000840b
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x0000843b
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x00008469
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x000084b5
--define_symbol gatt_client_register_handler=0x00008501
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x0000850d
--define_symbol gatt_client_signed_write_without_response=0x0000893d
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008a01
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008a3b
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008a8d
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008a9d
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008ad9
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008ae9
--define_symbol gatt_client_write_value_of_characteristic=0x00008b25
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00008b5b
--define_symbol hci_add_event_handler=0x0000a081
--define_symbol hci_power_control=0x0000a81d
--define_symbol hci_register_acl_packet_handler=0x0000a9d1
--define_symbol kv_commit=0x0000b16d
--define_symbol kv_get=0x0000b1c9
--define_symbol kv_init=0x0000b1d5
--define_symbol kv_init_backend=0x0000b255
--define_symbol kv_put=0x0000b269
--define_symbol kv_remove=0x0000b275
--define_symbol kv_remove_all=0x0000b2a9
--define_symbol kv_value_modified=0x0000b2d9
--define_symbol kv_value_modified_of_key=0x0000b2f5
--define_symbol kv_visit=0x0000b301
--define_symbol l2cap_add_event_handler=0x0000b391
--define_symbol l2cap_can_send_packet_now=0x0000b3a1
--define_symbol l2cap_create_le_credit_based_connection_request=0x0000b55d
--define_symbol l2cap_credit_based_send=0x0000b6a1
--define_symbol l2cap_credit_based_send_continue=0x0000b6cd
--define_symbol l2cap_disconnect=0x0000b749
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0000b999
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0000b9b5
--define_symbol l2cap_init=0x0000bd89
--define_symbol l2cap_le_send_flow_control_credit=0x0000be7f
--define_symbol l2cap_max_le_mtu=0x0000c189
--define_symbol l2cap_register_packet_handler=0x0000c2b1
--define_symbol l2cap_register_service=0x0000c2bd
--define_symbol l2cap_request_can_send_now_event=0x0000c3cd
--define_symbol l2cap_request_connection_parameter_update=0x0000c3e7
--define_symbol l2cap_send_echo_request=0x0000c8c1
--define_symbol l2cap_unregister_service=0x0000c981
--define_symbol le_device_db_add=0x0000c9d9
--define_symbol le_device_db_find=0x0000cab1
--define_symbol le_device_db_from_key=0x0000cadd
--define_symbol le_device_db_iter_cur=0x0000cae5
--define_symbol le_device_db_iter_cur_key=0x0000cae9
--define_symbol le_device_db_iter_init=0x0000caed
--define_symbol le_device_db_iter_next=0x0000caf5
--define_symbol le_device_db_remove_key=0x0000cb1b
--define_symbol ll_aes_encrypt=0x0000cb49
--define_symbol ll_config=0x0000cbc5
--define_symbol ll_free=0x0000cbfb
--define_symbol ll_get_heap_free_size=0x0000cc05
--define_symbol ll_hint_on_ce_len=0x0000cc19
--define_symbol ll_legacy_adv_set_interval=0x0000cc51
--define_symbol ll_malloc=0x0000cc61
--define_symbol ll_query_timing_info=0x0000cd99
--define_symbol ll_register_hci_acl_previewer=0x0000cde5
--define_symbol ll_scan_set_fixed_channel=0x0000ce49
--define_symbol ll_set_adv_access_address=0x0000d061
--define_symbol ll_set_adv_coded_scheme=0x0000d06d
--define_symbol ll_set_conn_acl_report_latency=0x0000d09d
--define_symbol ll_set_conn_coded_scheme=0x0000d0cd
--define_symbol ll_set_conn_latency=0x0000d0f9
--define_symbol ll_set_conn_tx_power=0x0000d129
--define_symbol ll_set_def_antenna=0x0000d171
--define_symbol ll_set_initiating_coded_scheme=0x0000d18d
--define_symbol ll_set_max_conn_number=0x0000d199
--define_symbol nibble_for_char=0x0001d325
--define_symbol platform_calibrate_rt_clk=0x0001d3bf
--define_symbol platform_call_on_stack=0x000040ef
--define_symbol platform_cancel_us_timer=0x0001d3c3
--define_symbol platform_config=0x0001d3d9
--define_symbol platform_controller_run=0x0001d4fd
--define_symbol platform_create_us_timer=0x0001d531
--define_symbol platform_delete_timer=0x0001d545
--define_symbol platform_enable_irq=0x0001d54d
--define_symbol platform_get_gen_os_driver=0x0001d585
--define_symbol platform_get_link_layer_interf=0x0001d591
--define_symbol platform_get_task_handle=0x0001d599
--define_symbol platform_get_timer_counter=0x0001d5b1
--define_symbol platform_get_us_time=0x0001d5b5
--define_symbol platform_get_version=0x0001d5b9
--define_symbol platform_hrng=0x0001d5c1
--define_symbol platform_init_controller=0x0001d5c9
--define_symbol platform_os_idle_resumed_hook=0x0001d5e5
--define_symbol platform_patch_rf_init_data=0x0001d5e9
--define_symbol platform_post_sleep_processing=0x0001d5f5
--define_symbol platform_pre_sleep_processing=0x0001d5fb
--define_symbol platform_pre_suppress_ticks_and_sleep_processing=0x0001d601
--define_symbol platform_printf=0x0001d605
--define_symbol platform_raise_assertion=0x0001d619
--define_symbol platform_rand=0x0001d62d
--define_symbol platform_read_info=0x0001d631
--define_symbol platform_read_persistent_reg=0x0001d661
--define_symbol platform_reset=0x0001d671
--define_symbol platform_rt_rc_auto_tune=0x0001d695
--define_symbol platform_rt_rc_auto_tune2=0x0001d69d
--define_symbol platform_rt_rc_tune=0x0001d725
--define_symbol platform_set_abs_timer=0x0001d739
--define_symbol platform_set_evt_callback=0x0001d73d
--define_symbol platform_set_evt_callback_table=0x0001d751
--define_symbol platform_set_irq_callback=0x0001d75d
--define_symbol platform_set_irq_callback_table=0x0001d779
--define_symbol platform_set_rf_clk_source=0x0001d785
--define_symbol platform_set_rf_init_data=0x0001d791
--define_symbol platform_set_rf_power_mapping=0x0001d79d
--define_symbol platform_set_timer=0x0001d7a9
--define_symbol platform_shutdown=0x0001d7ad
--define_symbol platform_switch_app=0x0001d7b1
--define_symbol platform_trace_raw=0x0001d7dd
--define_symbol platform_write_persistent_reg=0x0001d7f5
--define_symbol printf_hexdump=0x0001d805
--define_symbol reverse_128=0x0001db45
--define_symbol reverse_24=0x0001db4b
--define_symbol reverse_256=0x0001db51
--define_symbol reverse_48=0x0001db57
--define_symbol reverse_56=0x0001db5d
--define_symbol reverse_64=0x0001db63
--define_symbol reverse_bd_addr=0x0001db69
--define_symbol reverse_bytes=0x0001db6f
--define_symbol sm_add_event_handler=0x0001dd0d
--define_symbol sm_address_resolution_lookup=0x0001de65
--define_symbol sm_authenticated=0x0001e20d
--define_symbol sm_authorization_decline=0x0001e21b
--define_symbol sm_authorization_grant=0x0001e23b
--define_symbol sm_authorization_state=0x0001e25b
--define_symbol sm_bonding_decline=0x0001e275
--define_symbol sm_config=0x0001e6d1
--define_symbol sm_config_conn=0x0001e705
--define_symbol sm_encryption_key_size=0x0001e8bf
--define_symbol sm_just_works_confirm=0x0001ee45
--define_symbol sm_le_device_key=0x0001f185
--define_symbol sm_passkey_input=0x0001f21b
--define_symbol sm_private_random_address_generation_get=0x0001f5d5
--define_symbol sm_private_random_address_generation_get_mode=0x0001f5dd
--define_symbol sm_private_random_address_generation_set_mode=0x0001f5e9
--define_symbol sm_private_random_address_generation_set_update_period=0x0001f611
--define_symbol sm_register_external_ltk_callback=0x0001f74d
--define_symbol sm_register_oob_data_callback=0x0001f759
--define_symbol sm_request_pairing=0x0001f765
--define_symbol sm_send_security_request=0x00020243
--define_symbol sm_set_accepted_stk_generation_methods=0x00020269
--define_symbol sm_set_authentication_requirements=0x00020275
--define_symbol sm_set_encryption_key_size_range=0x00020285
--define_symbol sscanf_bd_addr=0x000205d1
--define_symbol sysSetPublicDeviceAddr=0x00020985
--define_symbol uuid128_to_str=0x00020f99
--define_symbol uuid_add_bluetooth_prefix=0x00020ff1
--define_symbol uuid_has_bluetooth_prefix=0x00021011
