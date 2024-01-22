--define_symbol att_dispatch_client_can_send_now=0x00005779
--define_symbol att_dispatch_client_request_can_send_now_event=0x0000577f
--define_symbol att_dispatch_register_client=0x00005785
--define_symbol att_dispatch_register_server=0x00005799
--define_symbol att_dispatch_server_can_send_now=0x000057ad
--define_symbol att_dispatch_server_request_can_send_now_event=0x000057b3
--define_symbol att_emit_general_event=0x00005865
--define_symbol att_server_can_send_packet_now=0x00005f91
--define_symbol att_server_deferred_read_response=0x00005f95
--define_symbol att_server_get_mtu=0x00005fad
--define_symbol att_server_indicate=0x00006019
--define_symbol att_server_init=0x0000609d
--define_symbol att_server_notify=0x000060d9
--define_symbol att_server_register_packet_handler=0x000061f1
--define_symbol att_server_request_can_send_now_event=0x000061fd
--define_symbol att_set_db=0x00006219
--define_symbol att_set_read_callback=0x0000622d
--define_symbol att_set_write_callback=0x00006239
--define_symbol bd_addr_cmp=0x000063a9
--define_symbol bd_addr_copy=0x000063af
--define_symbol bd_addr_to_str=0x000063b9
--define_symbol big_endian_read_16=0x000063f1
--define_symbol big_endian_read_32=0x000063f9
--define_symbol big_endian_store_16=0x0000640d
--define_symbol big_endian_store_32=0x00006419
--define_symbol btstack_config=0x00006551
--define_symbol btstack_memory_pool_create=0x0000669f
--define_symbol btstack_memory_pool_free=0x000066c9
--define_symbol btstack_memory_pool_get=0x00006729
--define_symbol btstack_push_user_msg=0x00006771
--define_symbol btstack_push_user_runnable=0x0000677d
--define_symbol btstack_reset=0x00006789
--define_symbol char_for_nibble=0x00006a4d
--define_symbol gap_add_dev_to_periodic_list=0x0000735d
--define_symbol gap_add_whitelist=0x0000736d
--define_symbol gap_aes_encrypt=0x00007379
--define_symbol gap_clear_white_lists=0x000073b1
--define_symbol gap_clr_adv_set=0x000073bd
--define_symbol gap_clr_periodic_adv_list=0x000073c9
--define_symbol gap_create_connection_cancel=0x000073d5
--define_symbol gap_default_periodic_adv_sync_transfer_param=0x000073e1
--define_symbol gap_disconnect=0x000073f9
--define_symbol gap_disconnect_all=0x00007425
--define_symbol gap_ext_create_connection=0x00007465
--define_symbol gap_get_connection_parameter_range=0x0000753d
--define_symbol gap_le_read_channel_map=0x00007575
--define_symbol gap_periodic_adv_create_sync=0x000075e1
--define_symbol gap_periodic_adv_create_sync_cancel=0x00007605
--define_symbol gap_periodic_adv_set_info_transfer=0x00007611
--define_symbol gap_periodic_adv_sync_transfer=0x00007621
--define_symbol gap_periodic_adv_sync_transfer_param=0x00007631
--define_symbol gap_periodic_adv_term_sync=0x0000764d
--define_symbol gap_read_antenna_info=0x000076d5
--define_symbol gap_read_periodic_adv_list_size=0x000076e1
--define_symbol gap_read_phy=0x000076ed
--define_symbol gap_read_remote_used_features=0x000076f9
--define_symbol gap_read_remote_version=0x00007705
--define_symbol gap_read_rssi=0x00007711
--define_symbol gap_remove_whitelist=0x0000771d
--define_symbol gap_rmv_adv_set=0x00007799
--define_symbol gap_rmv_dev_from_periodic_list=0x000077a5
--define_symbol gap_rx_test_v2=0x000077b5
--define_symbol gap_rx_test_v3=0x000077c5
--define_symbol gap_set_adv_set_random_addr=0x00007811
--define_symbol gap_set_callback_for_next_hci=0x0000784d
--define_symbol gap_set_connection_cte_request_enable=0x0000786d
--define_symbol gap_set_connection_cte_response_enable=0x00007889
--define_symbol gap_set_connection_cte_rx_param=0x00007899
--define_symbol gap_set_connection_cte_tx_param=0x000078ed
--define_symbol gap_set_connection_parameter_range=0x00007939
--define_symbol gap_set_connectionless_cte_tx_enable=0x00007951
--define_symbol gap_set_connectionless_cte_tx_param=0x00007961
--define_symbol gap_set_connectionless_iq_sampling_enable=0x000079bd
--define_symbol gap_set_data_length=0x00007a19
--define_symbol gap_set_def_phy=0x00007a31
--define_symbol gap_set_ext_adv_data=0x00007a41
--define_symbol gap_set_ext_adv_enable=0x00007a59
--define_symbol gap_set_ext_adv_para=0x00007ac9
--define_symbol gap_set_ext_scan_enable=0x00007ba1
--define_symbol gap_set_ext_scan_para=0x00007bb9
--define_symbol gap_set_ext_scan_response_data=0x00007c59
--define_symbol gap_set_host_channel_classification=0x00007c71
--define_symbol gap_set_periodic_adv_data=0x00007c81
--define_symbol gap_set_periodic_adv_enable=0x00007cf1
--define_symbol gap_set_periodic_adv_para=0x00007d01
--define_symbol gap_set_periodic_adv_rx_enable=0x00007d19
--define_symbol gap_set_phy=0x00007d29
--define_symbol gap_set_random_device_address=0x00007d45
--define_symbol gap_start_ccm=0x00007d75
--define_symbol gap_test_end=0x00007dbd
--define_symbol gap_tx_test_v2=0x00007dc9
--define_symbol gap_tx_test_v4=0x00007de1
--define_symbol gap_update_connection_parameters=0x00007e05
--define_symbol gap_vendor_tx_continuous_wave=0x00007e45
--define_symbol gatt_client_cancel_write=0x0000836d
--define_symbol gatt_client_discover_characteristic_descriptors=0x00008393
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x000083d3
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x00008423
--define_symbol gatt_client_discover_characteristics_for_service=0x00008473
--define_symbol gatt_client_discover_primary_services=0x000084a9
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x000084db
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x0000851f
--define_symbol gatt_client_execute_write=0x0000855b
--define_symbol gatt_client_find_included_services_for_service=0x00008581
--define_symbol gatt_client_get_mtu=0x000085af
--define_symbol gatt_client_is_ready=0x0000864d
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x00008663
--define_symbol gatt_client_prepare_write=0x00008683
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x000086bf
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x000086e9
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x000086ef
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x0000871d
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x00008723
--define_symbol gatt_client_read_multiple_characteristic_values=0x00008751
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x00008781
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x000087af
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x000087fb
--define_symbol gatt_client_register_handler=0x00008845
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x00008851
--define_symbol gatt_client_signed_write_without_response=0x00008c55
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008d19
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008d53
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008da5
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008db5
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008df1
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008e01
--define_symbol gatt_client_write_value_of_characteristic=0x00008e3d
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00008e73
--define_symbol hci_add_event_handler=0x0000a3a9
--define_symbol hci_power_control=0x0000ab31
--define_symbol hci_register_acl_packet_handler=0x0000ace5
--define_symbol kv_commit=0x0000b475
--define_symbol kv_get=0x0000b4d1
--define_symbol kv_init=0x0000b4dd
--define_symbol kv_init_backend=0x0000b559
--define_symbol kv_put=0x0000b56d
--define_symbol kv_remove=0x0000b579
--define_symbol kv_remove_all=0x0000b5ad
--define_symbol kv_value_modified=0x0000b5dd
--define_symbol kv_value_modified_of_key=0x0000b5f9
--define_symbol kv_visit=0x0000b605
--define_symbol l2cap_add_event_handler=0x0000b695
--define_symbol l2cap_can_send_packet_now=0x0000b6a5
--define_symbol l2cap_create_le_credit_based_connection_request=0x0000b861
--define_symbol l2cap_credit_based_send=0x0000b9a5
--define_symbol l2cap_credit_based_send_continue=0x0000b9d1
--define_symbol l2cap_disconnect=0x0000ba4d
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0000bc9d
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0000bcb9
--define_symbol l2cap_init=0x0000c08d
--define_symbol l2cap_le_send_flow_control_credit=0x0000c183
--define_symbol l2cap_max_le_mtu=0x0000c48d
--define_symbol l2cap_register_packet_handler=0x0000c5b5
--define_symbol l2cap_register_service=0x0000c5c1
--define_symbol l2cap_request_can_send_now_event=0x0000c6d1
--define_symbol l2cap_request_connection_parameter_update=0x0000c6eb
--define_symbol l2cap_send_echo_request=0x0000cbc5
--define_symbol l2cap_unregister_service=0x0000cc85
--define_symbol le_device_db_add=0x0000ccdd
--define_symbol le_device_db_find=0x0000cda5
--define_symbol le_device_db_from_key=0x0000cdd1
--define_symbol le_device_db_iter_cur=0x0000cdd9
--define_symbol le_device_db_iter_cur_key=0x0000cddd
--define_symbol le_device_db_iter_init=0x0000cde1
--define_symbol le_device_db_iter_next=0x0000cde9
--define_symbol le_device_db_remove_key=0x0000ce0f
--define_symbol ll_aes_encrypt=0x0000ce3d
--define_symbol ll_config=0x0000ceb9
--define_symbol ll_free=0x0000ceef
--define_symbol ll_get_heap_free_size=0x0000cef9
--define_symbol ll_hint_on_ce_len=0x0000cf0d
--define_symbol ll_legacy_adv_set_interval=0x0000cf45
--define_symbol ll_malloc=0x0000cf55
--define_symbol ll_query_timing_info=0x0000d08d
--define_symbol ll_register_hci_acl_previewer=0x0000d0d9
--define_symbol ll_scan_set_fixed_channel=0x0000d13d
--define_symbol ll_set_adv_access_address=0x0000d355
--define_symbol ll_set_adv_coded_scheme=0x0000d361
--define_symbol ll_set_conn_acl_report_latency=0x0000d389
--define_symbol ll_set_conn_coded_scheme=0x0000d3b5
--define_symbol ll_set_conn_latency=0x0000d3dd
--define_symbol ll_set_conn_tx_power=0x0000d409
--define_symbol ll_set_def_antenna=0x0000d44d
--define_symbol ll_set_initiating_coded_scheme=0x0000d469
--define_symbol ll_set_max_conn_number=0x0000d475
--define_symbol nibble_for_char=0x0001e3b1
--define_symbol platform_32k_rc_auto_tune=0x0001e441
--define_symbol platform_32k_rc_tune=0x0001e4bd
--define_symbol platform_calibrate_32k=0x0001e4d1
--define_symbol platform_config=0x0001e4d5
--define_symbol platform_controller_run=0x0001e5f9
--define_symbol platform_delete_timer=0x0001e62d
--define_symbol platform_enable_irq=0x0001e635
--define_symbol platform_get_gen_os_driver=0x0001e66d
--define_symbol platform_get_link_layer_interf=0x0001e679
--define_symbol platform_get_task_handle=0x0001e681
--define_symbol platform_get_timer_counter=0x0001e699
--define_symbol platform_get_us_time=0x0001e69d
--define_symbol platform_get_version=0x0001e6a1
--define_symbol platform_hrng=0x0001e6a9
--define_symbol platform_init_controller=0x0001e6b1
--define_symbol platform_os_idle_resumed_hook=0x0001e6cd
--define_symbol platform_patch_rf_init_data=0x0001e6d1
--define_symbol platform_post_sleep_processing=0x0001e6dd
--define_symbol platform_pre_sleep_processing=0x0001e6e3
--define_symbol platform_pre_suppress_ticks_and_sleep_processing=0x0001e6e9
--define_symbol platform_printf=0x0001e6ed
--define_symbol platform_raise_assertion=0x0001e701
--define_symbol platform_rand=0x0001e715
--define_symbol platform_read_info=0x0001e719
--define_symbol platform_read_persistent_reg=0x0001e749
--define_symbol platform_reset=0x0001e759
--define_symbol platform_set_abs_timer=0x0001e77d
--define_symbol platform_set_evt_callback=0x0001e781
--define_symbol platform_set_evt_callback_table=0x0001e795
--define_symbol platform_set_irq_callback=0x0001e7a1
--define_symbol platform_set_irq_callback_table=0x0001e7bd
--define_symbol platform_set_rf_clk_source=0x0001e7c9
--define_symbol platform_set_rf_init_data=0x0001e7d5
--define_symbol platform_set_rf_power_mapping=0x0001e7e1
--define_symbol platform_set_timer=0x0001e7ed
--define_symbol platform_shutdown=0x0001e7f1
--define_symbol platform_switch_app=0x0001e7f5
--define_symbol platform_trace_raw=0x0001e821
--define_symbol platform_write_persistent_reg=0x0001e839
--define_symbol printf_hexdump=0x0001e849
--define_symbol reverse_128=0x0001ebb9
--define_symbol reverse_24=0x0001ebbf
--define_symbol reverse_256=0x0001ebc5
--define_symbol reverse_48=0x0001ebcb
--define_symbol reverse_56=0x0001ebd1
--define_symbol reverse_64=0x0001ebd7
--define_symbol reverse_bd_addr=0x0001ebdd
--define_symbol reverse_bytes=0x0001ebe3
--define_symbol sm_add_event_handler=0x0001eed1
--define_symbol sm_address_resolution_lookup=0x0001f029
--define_symbol sm_authenticated=0x0001f3a5
--define_symbol sm_authorization_decline=0x0001f3b3
--define_symbol sm_authorization_grant=0x0001f3d3
--define_symbol sm_authorization_state=0x0001f3f3
--define_symbol sm_bonding_decline=0x0001f40d
--define_symbol sm_config=0x0001f869
--define_symbol sm_config_conn=0x0001f881
--define_symbol sm_encryption_key_size=0x0001fa37
--define_symbol sm_just_works_confirm=0x0001ffbd
--define_symbol sm_le_device_key=0x00020309
--define_symbol sm_passkey_input=0x0002039f
--define_symbol sm_private_random_address_generation_get=0x00020759
--define_symbol sm_private_random_address_generation_get_mode=0x00020761
--define_symbol sm_private_random_address_generation_set_mode=0x0002076d
--define_symbol sm_private_random_address_generation_set_update_period=0x00020795
--define_symbol sm_register_external_ltk_callback=0x000208d1
--define_symbol sm_register_oob_data_callback=0x000208dd
--define_symbol sm_request_pairing=0x000208e9
--define_symbol sm_send_security_request=0x000213bf
--define_symbol sm_set_accepted_stk_generation_methods=0x000213e5
--define_symbol sm_set_authentication_requirements=0x000213f1
--define_symbol sm_set_encryption_key_size_range=0x000213fd
--define_symbol sscanf_bd_addr=0x00021759
--define_symbol sysSetPublicDeviceAddr=0x00021b0d
--define_symbol uuid128_to_str=0x000222ad
--define_symbol uuid_add_bluetooth_prefix=0x00022305
--define_symbol uuid_has_bluetooth_prefix=0x00022325
