--define_symbol att_dispatch_client_can_send_now=0x0200508d
--define_symbol att_dispatch_client_request_can_send_now_event=0x02005093
--define_symbol att_dispatch_register_client=0x02005099
--define_symbol att_dispatch_register_server=0x020050ad
--define_symbol att_dispatch_server_can_send_now=0x020050c1
--define_symbol att_dispatch_server_request_can_send_now_event=0x020050c7
--define_symbol att_emit_general_event=0x02005179
--define_symbol att_server_can_send_packet_now=0x020058b1
--define_symbol att_server_deferred_read_response=0x020058b5
--define_symbol att_server_get_mtu=0x020058cd
--define_symbol att_server_indicate=0x02005939
--define_symbol att_server_init=0x020059bd
--define_symbol att_server_notify=0x020059f9
--define_symbol att_server_register_packet_handler=0x02005b11
--define_symbol att_server_request_can_send_now_event=0x02005b1d
--define_symbol att_set_db=0x02005b39
--define_symbol att_set_read_callback=0x02005b4d
--define_symbol att_set_write_callback=0x02005b59
--define_symbol bd_addr_cmp=0x02005d29
--define_symbol bd_addr_copy=0x02005d2f
--define_symbol bd_addr_to_str=0x02005d39
--define_symbol big_endian_read_16=0x02005d71
--define_symbol big_endian_read_32=0x02005d79
--define_symbol big_endian_store_16=0x02005d8d
--define_symbol big_endian_store_32=0x02005d99
--define_symbol btstack_config=0x02005dc5
--define_symbol btstack_memory_pool_create=0x02005f13
--define_symbol btstack_memory_pool_free=0x02005f3d
--define_symbol btstack_memory_pool_get=0x02005f9d
--define_symbol btstack_push_user_msg=0x02005fe5
--define_symbol btstack_push_user_runnable=0x02005ff1
--define_symbol btstack_reset=0x02005ffd
--define_symbol char_for_nibble=0x0200620b
--define_symbol gap_add_dev_to_periodic_list=0x02006b79
--define_symbol gap_add_whitelist=0x02006b89
--define_symbol gap_aes_encrypt=0x02006b95
--define_symbol gap_clear_white_lists=0x02006bcd
--define_symbol gap_clr_adv_set=0x02006bd9
--define_symbol gap_clr_periodic_adv_list=0x02006be5
--define_symbol gap_create_connection_cancel=0x02006bf1
--define_symbol gap_default_periodic_adv_sync_transfer_param=0x02006bfd
--define_symbol gap_disconnect=0x02006c15
--define_symbol gap_disconnect_all=0x02006c41
--define_symbol gap_ext_create_connection=0x02006c81
--define_symbol gap_get_connection_parameter_range=0x02006d59
--define_symbol gap_le_read_channel_map=0x02006d91
--define_symbol gap_periodic_adv_create_sync=0x02006dfd
--define_symbol gap_periodic_adv_create_sync_cancel=0x02006e21
--define_symbol gap_periodic_adv_set_info_transfer=0x02006e2d
--define_symbol gap_periodic_adv_sync_transfer=0x02006e3d
--define_symbol gap_periodic_adv_sync_transfer_param=0x02006e4d
--define_symbol gap_periodic_adv_term_sync=0x02006e69
--define_symbol gap_read_antenna_info=0x02006ef1
--define_symbol gap_read_periodic_adv_list_size=0x02006efd
--define_symbol gap_read_phy=0x02006f09
--define_symbol gap_read_remote_used_features=0x02006f15
--define_symbol gap_read_remote_version=0x02006f21
--define_symbol gap_read_rssi=0x02006f2d
--define_symbol gap_remove_whitelist=0x02006f39
--define_symbol gap_rmv_adv_set=0x02006fb5
--define_symbol gap_rmv_dev_from_periodic_list=0x02006fc1
--define_symbol gap_rx_test_v2=0x02006fd1
--define_symbol gap_rx_test_v3=0x02006fe1
--define_symbol gap_set_adv_set_random_addr=0x0200702d
--define_symbol gap_set_callback_for_next_hci=0x02007069
--define_symbol gap_set_connection_cte_request_enable=0x02007089
--define_symbol gap_set_connection_cte_response_enable=0x020070a5
--define_symbol gap_set_connection_cte_rx_param=0x020070b5
--define_symbol gap_set_connection_cte_tx_param=0x02007109
--define_symbol gap_set_connection_parameter_range=0x02007155
--define_symbol gap_set_connectionless_cte_tx_enable=0x0200716d
--define_symbol gap_set_connectionless_cte_tx_param=0x0200717d
--define_symbol gap_set_connectionless_iq_sampling_enable=0x020071d9
--define_symbol gap_set_data_length=0x02007235
--define_symbol gap_set_def_phy=0x0200724d
--define_symbol gap_set_ext_adv_data=0x0200725d
--define_symbol gap_set_ext_adv_enable=0x02007275
--define_symbol gap_set_ext_adv_para=0x020072e5
--define_symbol gap_set_ext_scan_enable=0x020073b5
--define_symbol gap_set_ext_scan_para=0x020073cd
--define_symbol gap_set_ext_scan_response_data=0x0200746d
--define_symbol gap_set_host_channel_classification=0x02007485
--define_symbol gap_set_periodic_adv_data=0x02007495
--define_symbol gap_set_periodic_adv_enable=0x02007505
--define_symbol gap_set_periodic_adv_para=0x02007515
--define_symbol gap_set_periodic_adv_rx_enable=0x0200752d
--define_symbol gap_set_phy=0x0200753d
--define_symbol gap_set_random_device_address=0x02007559
--define_symbol gap_start_ccm=0x02007589
--define_symbol gap_test_end=0x020075d1
--define_symbol gap_tx_test_v2=0x020075dd
--define_symbol gap_tx_test_v4=0x020075f5
--define_symbol gap_update_connection_parameters=0x02007619
--define_symbol gap_vendor_tx_continuous_wave=0x02007659
--define_symbol gatt_client_cancel_write=0x02007b81
--define_symbol gatt_client_discover_characteristic_descriptors=0x02007ba7
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x02007be7
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x02007c37
--define_symbol gatt_client_discover_characteristics_for_service=0x02007c87
--define_symbol gatt_client_discover_primary_services=0x02007cbd
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x02007cef
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x02007d33
--define_symbol gatt_client_execute_write=0x02007d71
--define_symbol gatt_client_find_included_services_for_service=0x02007d97
--define_symbol gatt_client_get_mtu=0x02007dc5
--define_symbol gatt_client_is_ready=0x02007e89
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x02007e9f
--define_symbol gatt_client_prepare_write=0x02007ebf
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x02007efb
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x02007f25
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x02007f2b
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x02007f59
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x02007f5f
--define_symbol gatt_client_read_multiple_characteristic_values=0x02007f8d
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x02007fbd
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x02007feb
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x02008037
--define_symbol gatt_client_register_handler=0x02008081
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x0200808d
--define_symbol gatt_client_signed_write_without_response=0x02008495
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x02008559
--define_symbol gatt_client_write_client_characteristic_configuration=0x02008593
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x020085e5
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x020085f5
--define_symbol gatt_client_write_long_value_of_characteristic=0x02008631
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x02008641
--define_symbol gatt_client_write_value_of_characteristic=0x0200867d
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x020086b3
--define_symbol hci_add_event_handler=0x02009ca9
--define_symbol hci_power_control=0x0200a43d
--define_symbol hci_register_acl_packet_handler=0x0200a5f1
--define_symbol kv_commit=0x0200ac75
--define_symbol kv_get=0x0200acd1
--define_symbol kv_init=0x0200acdd
--define_symbol kv_init_backend=0x0200ad5d
--define_symbol kv_put=0x0200ad71
--define_symbol kv_remove=0x0200ad7d
--define_symbol kv_remove_all=0x0200adb1
--define_symbol kv_value_modified=0x0200ade1
--define_symbol kv_value_modified_of_key=0x0200adfd
--define_symbol kv_visit=0x0200ae09
--define_symbol l2cap_add_event_handler=0x0200ae99
--define_symbol l2cap_can_send_packet_now=0x0200aea9
--define_symbol l2cap_create_le_credit_based_connection_request=0x0200b06d
--define_symbol l2cap_credit_based_send=0x0200b1b5
--define_symbol l2cap_credit_based_send_continue=0x0200b1e1
--define_symbol l2cap_disconnect=0x0200b25d
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0200b449
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0200b465
--define_symbol l2cap_init=0x0200b8a5
--define_symbol l2cap_le_send_flow_control_credit=0x0200b99b
--define_symbol l2cap_max_le_mtu=0x0200bca9
--define_symbol l2cap_register_packet_handler=0x0200bdd1
--define_symbol l2cap_register_service=0x0200bddd
--define_symbol l2cap_request_can_send_now_event=0x0200beed
--define_symbol l2cap_request_connection_parameter_update=0x0200bf07
--define_symbol l2cap_send_echo_request=0x0200c3e9
--define_symbol l2cap_unregister_service=0x0200c4c9
--define_symbol le_device_db_add=0x0200c521
--define_symbol le_device_db_find=0x0200c5e9
--define_symbol le_device_db_from_key=0x0200c615
--define_symbol le_device_db_iter_cur=0x0200c61d
--define_symbol le_device_db_iter_cur_key=0x0200c621
--define_symbol le_device_db_iter_init=0x0200c625
--define_symbol le_device_db_iter_next=0x0200c62d
--define_symbol le_device_db_remove_key=0x0200c653
--define_symbol ll_adjust_conn_peer_tx_power=0x0200c681
--define_symbol ll_aes_encrypt=0x0200c6a9
--define_symbol ll_config=0x0200c71d
--define_symbol ll_free=0x0200c753
--define_symbol ll_get_heap_free_size=0x0200c75d
--define_symbol ll_hint_on_ce_len=0x0200c771
--define_symbol ll_legacy_adv_set_interval=0x0200c7a9
--define_symbol ll_malloc=0x0200c7b9
--define_symbol ll_register_hci_acl_previewer=0x0200c8d1
--define_symbol ll_scan_set_fixed_channel=0x0200c935
--define_symbol ll_set_adv_access_address=0x0200cb4d
--define_symbol ll_set_adv_coded_scheme=0x0200cb59
--define_symbol ll_set_conn_acl_report_latency=0x0200cb89
--define_symbol ll_set_conn_coded_scheme=0x0200cbb5
--define_symbol ll_set_conn_latency=0x0200cbdd
--define_symbol ll_set_conn_tx_power=0x0200cc09
--define_symbol ll_set_def_antenna=0x0200cc41
--define_symbol ll_set_initiating_coded_scheme=0x0200cc61
--define_symbol ll_set_max_conn_number=0x0200cc6d
--define_symbol ll_set_tx_power_range=0x0200cd01
--define_symbol nibble_for_char=0x0201ce3d
--define_symbol platform_calibrate_rt_clk=0x0201ceb3
--define_symbol platform_call_on_stack=0x0200303f
--define_symbol platform_cancel_us_timer=0x0201ceb7
--define_symbol platform_config=0x0201cecd
--define_symbol platform_controller_run=0x2000015b
--define_symbol platform_create_us_timer=0x0201d001
--define_symbol platform_delete_timer=0x0201d015
--define_symbol platform_enable_irq=0x0201d01d
--define_symbol platform_get_gen_os_driver=0x0201d071
--define_symbol platform_get_link_layer_interf=0x0201d07d
--define_symbol platform_get_task_handle=0x0201d085
--define_symbol platform_get_timer_counter=0x0201d09d
--define_symbol platform_get_us_time=0x0201d0a1
--define_symbol platform_get_version=0x0201d0a5
--define_symbol platform_hrng=0x0201d0c1
--define_symbol platform_init_controller=0x20000145
--define_symbol platform_os_idle_resumed_hook=0x200003df
--define_symbol platform_patch_rf_init_data=0x0201d0c9
--define_symbol platform_post_sleep_processing=0x200003d9
--define_symbol platform_pre_sleep_processing=0x200003d3
--define_symbol platform_pre_suppress_ticks_and_sleep_processing=0x200003cf
--define_symbol platform_printf=0x0201d0d5
--define_symbol platform_raise_assertion=0x0201d0e9
--define_symbol platform_rand=0x0201d0fd
--define_symbol platform_read_info=0x0201d101
--define_symbol platform_read_persistent_reg=0x0201d131
--define_symbol platform_reset=0x0201d13d
--define_symbol platform_rt_rc_auto_tune=0x0201d151
--define_symbol platform_rt_rc_auto_tune2=0x0201d159
--define_symbol platform_rt_rc_tune=0x0201d1c1
--define_symbol platform_set_abs_timer=0x0201d1dd
--define_symbol platform_set_evt_callback=0x0201d1e1
--define_symbol platform_set_evt_callback_table=0x0201d1f5
--define_symbol platform_set_irq_callback=0x0201d201
--define_symbol platform_set_irq_callback_table=0x0201d21d
--define_symbol platform_set_rf_clk_source=0x0201d229
--define_symbol platform_set_rf_init_data=0x0201d235
--define_symbol platform_set_rf_power_mapping=0x0201d241
--define_symbol platform_set_timer=0x0201d24d
--define_symbol platform_shutdown=0x0201d251
--define_symbol platform_switch_app=0x0201d255
--define_symbol platform_trace_raw=0x0201d26d
--define_symbol platform_write_persistent_reg=0x0201d285
--define_symbol printf_hexdump=0x0201d299
--define_symbol reverse_128=0x0201d599
--define_symbol reverse_24=0x0201d59f
--define_symbol reverse_256=0x0201d5a5
--define_symbol reverse_48=0x0201d5ab
--define_symbol reverse_56=0x0201d5b1
--define_symbol reverse_64=0x0201d5b7
--define_symbol reverse_bd_addr=0x0201d5bd
--define_symbol reverse_bytes=0x0201d5c3
--define_symbol sm_add_event_handler=0x0201d8c9
--define_symbol sm_address_resolution_lookup=0x0201da21
--define_symbol sm_authenticated=0x0201ddc9
--define_symbol sm_authorization_decline=0x0201ddd7
--define_symbol sm_authorization_grant=0x0201ddf7
--define_symbol sm_authorization_state=0x0201de17
--define_symbol sm_bonding_decline=0x0201de31
--define_symbol sm_config=0x0201e285
--define_symbol sm_config_conn=0x0201e2b9
--define_symbol sm_encryption_key_size=0x0201e473
--define_symbol sm_just_works_confirm=0x0201e9f9
--define_symbol sm_le_device_key=0x0201ed39
--define_symbol sm_passkey_input=0x0201edcf
--define_symbol sm_private_random_address_generation_get=0x0201f189
--define_symbol sm_private_random_address_generation_get_mode=0x0201f191
--define_symbol sm_private_random_address_generation_set_mode=0x0201f19d
--define_symbol sm_private_random_address_generation_set_update_period=0x0201f1c5
--define_symbol sm_register_external_ltk_callback=0x0201f301
--define_symbol sm_register_oob_data_callback=0x0201f30d
--define_symbol sm_request_pairing=0x0201f319
--define_symbol sm_send_security_request=0x0201fdf7
--define_symbol sm_set_accepted_stk_generation_methods=0x0201fe1d
--define_symbol sm_set_authentication_requirements=0x0201fe29
--define_symbol sm_set_encryption_key_size_range=0x0201fe39
--define_symbol sscanf_bd_addr=0x02020189
--define_symbol sysSetPublicDeviceAddr=0x0202023d
--define_symbol uuid128_to_str=0x020209f9
--define_symbol uuid_add_bluetooth_prefix=0x02020a51
--define_symbol uuid_has_bluetooth_prefix=0x02020a71
