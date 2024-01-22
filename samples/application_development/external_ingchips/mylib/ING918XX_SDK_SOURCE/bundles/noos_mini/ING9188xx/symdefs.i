--define_symbol att_dispatch_client_can_send_now=0x000057c1
--define_symbol att_dispatch_client_request_can_send_now_event=0x000057c7
--define_symbol att_dispatch_register_client=0x000057cd
--define_symbol att_dispatch_register_server=0x000057e1
--define_symbol att_dispatch_server_can_send_now=0x000057f5
--define_symbol att_dispatch_server_request_can_send_now_event=0x000057fb
--define_symbol att_emit_general_event=0x000058ad
--define_symbol att_server_can_send_packet_now=0x00005fdd
--define_symbol att_server_deferred_read_response=0x00005fe1
--define_symbol att_server_get_mtu=0x00005ff9
--define_symbol att_server_indicate=0x00006065
--define_symbol att_server_init=0x000060e9
--define_symbol att_server_notify=0x00006125
--define_symbol att_server_register_packet_handler=0x0000623d
--define_symbol att_server_request_can_send_now_event=0x00006249
--define_symbol att_set_db=0x00006265
--define_symbol att_set_read_callback=0x00006279
--define_symbol att_set_write_callback=0x00006285
--define_symbol bd_addr_cmp=0x000063f5
--define_symbol bd_addr_copy=0x000063fb
--define_symbol bd_addr_to_str=0x00006405
--define_symbol big_endian_read_16=0x0000643d
--define_symbol big_endian_read_32=0x00006445
--define_symbol big_endian_store_16=0x00006459
--define_symbol big_endian_store_32=0x00006465
--define_symbol btstack_config=0x0000659d
--define_symbol btstack_memory_pool_create=0x000066eb
--define_symbol btstack_memory_pool_free=0x00006715
--define_symbol btstack_memory_pool_get=0x00006775
--define_symbol btstack_push_user_msg=0x000067bd
--define_symbol btstack_push_user_runnable=0x000067c9
--define_symbol btstack_reset=0x000067d5
--define_symbol char_for_nibble=0x00006a99
--define_symbol gap_add_dev_to_periodic_list=0x000073ad
--define_symbol gap_add_whitelist=0x000073bd
--define_symbol gap_aes_encrypt=0x000073c9
--define_symbol gap_clear_white_lists=0x00007401
--define_symbol gap_clr_adv_set=0x0000740d
--define_symbol gap_clr_periodic_adv_list=0x00007419
--define_symbol gap_create_connection_cancel=0x00007425
--define_symbol gap_default_periodic_adv_sync_transfer_param=0x00007431
--define_symbol gap_disconnect=0x00007449
--define_symbol gap_disconnect_all=0x00007475
--define_symbol gap_ext_create_connection=0x000074b5
--define_symbol gap_get_connection_parameter_range=0x0000758d
--define_symbol gap_le_read_channel_map=0x000075c5
--define_symbol gap_periodic_adv_create_sync=0x00007631
--define_symbol gap_periodic_adv_create_sync_cancel=0x00007655
--define_symbol gap_periodic_adv_set_info_transfer=0x00007661
--define_symbol gap_periodic_adv_sync_transfer=0x00007671
--define_symbol gap_periodic_adv_sync_transfer_param=0x00007681
--define_symbol gap_periodic_adv_term_sync=0x0000769d
--define_symbol gap_read_antenna_info=0x00007725
--define_symbol gap_read_periodic_adv_list_size=0x00007731
--define_symbol gap_read_phy=0x0000773d
--define_symbol gap_read_remote_used_features=0x00007749
--define_symbol gap_read_remote_version=0x00007755
--define_symbol gap_read_rssi=0x00007761
--define_symbol gap_remove_whitelist=0x0000776d
--define_symbol gap_rmv_adv_set=0x000077e9
--define_symbol gap_rmv_dev_from_periodic_list=0x000077f5
--define_symbol gap_rx_test_v2=0x00007805
--define_symbol gap_rx_test_v3=0x00007815
--define_symbol gap_set_adv_set_random_addr=0x00007861
--define_symbol gap_set_callback_for_next_hci=0x0000789d
--define_symbol gap_set_connection_cte_request_enable=0x000078bd
--define_symbol gap_set_connection_cte_response_enable=0x000078d9
--define_symbol gap_set_connection_cte_rx_param=0x000078e9
--define_symbol gap_set_connection_cte_tx_param=0x0000793d
--define_symbol gap_set_connection_parameter_range=0x00007989
--define_symbol gap_set_connectionless_cte_tx_enable=0x000079a1
--define_symbol gap_set_connectionless_cte_tx_param=0x000079b1
--define_symbol gap_set_connectionless_iq_sampling_enable=0x00007a0d
--define_symbol gap_set_data_length=0x00007a69
--define_symbol gap_set_def_phy=0x00007a81
--define_symbol gap_set_ext_adv_data=0x00007a91
--define_symbol gap_set_ext_adv_enable=0x00007aa9
--define_symbol gap_set_ext_adv_para=0x00007b19
--define_symbol gap_set_ext_scan_enable=0x00007bf1
--define_symbol gap_set_ext_scan_para=0x00007c09
--define_symbol gap_set_ext_scan_response_data=0x00007ca9
--define_symbol gap_set_host_channel_classification=0x00007cc1
--define_symbol gap_set_periodic_adv_data=0x00007cd1
--define_symbol gap_set_periodic_adv_enable=0x00007d41
--define_symbol gap_set_periodic_adv_para=0x00007d51
--define_symbol gap_set_periodic_adv_rx_enable=0x00007d69
--define_symbol gap_set_phy=0x00007d79
--define_symbol gap_set_random_device_address=0x00007d95
--define_symbol gap_start_ccm=0x00007dc5
--define_symbol gap_test_end=0x00007e0d
--define_symbol gap_tx_test_v2=0x00007e19
--define_symbol gap_tx_test_v4=0x00007e31
--define_symbol gap_update_connection_parameters=0x00007e55
--define_symbol gap_vendor_tx_continuous_wave=0x00007e95
--define_symbol gatt_client_cancel_write=0x000083bd
--define_symbol gatt_client_discover_characteristic_descriptors=0x000083e3
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x00008423
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x00008473
--define_symbol gatt_client_discover_characteristics_for_service=0x000084c3
--define_symbol gatt_client_discover_primary_services=0x000084f9
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x0000852b
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x0000856f
--define_symbol gatt_client_execute_write=0x000085ab
--define_symbol gatt_client_find_included_services_for_service=0x000085d1
--define_symbol gatt_client_get_mtu=0x000085ff
--define_symbol gatt_client_is_ready=0x0000869d
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x000086b3
--define_symbol gatt_client_prepare_write=0x000086d3
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x0000870f
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x00008739
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x0000873f
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x0000876d
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x00008773
--define_symbol gatt_client_read_multiple_characteristic_values=0x000087a1
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x000087d1
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x000087ff
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x0000884b
--define_symbol gatt_client_register_handler=0x00008895
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x000088a1
--define_symbol gatt_client_signed_write_without_response=0x00008ca5
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008d69
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008da3
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008df5
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008e05
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008e41
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008e51
--define_symbol gatt_client_write_value_of_characteristic=0x00008e8d
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00008ec3
--define_symbol hci_add_event_handler=0x0000a3f9
--define_symbol hci_power_control=0x0000ab81
--define_symbol hci_register_acl_packet_handler=0x0000ad35
--define_symbol kv_commit=0x0000b4c5
--define_symbol kv_get=0x0000b521
--define_symbol kv_init=0x0000b52d
--define_symbol kv_init_backend=0x0000b5a9
--define_symbol kv_put=0x0000b5bd
--define_symbol kv_remove=0x0000b5c9
--define_symbol kv_remove_all=0x0000b5fd
--define_symbol kv_value_modified=0x0000b62d
--define_symbol kv_value_modified_of_key=0x0000b649
--define_symbol kv_visit=0x0000b655
--define_symbol l2cap_add_event_handler=0x0000b6e5
--define_symbol l2cap_can_send_packet_now=0x0000b6f5
--define_symbol l2cap_create_le_credit_based_connection_request=0x0000b8b1
--define_symbol l2cap_credit_based_send=0x0000b9f5
--define_symbol l2cap_credit_based_send_continue=0x0000ba21
--define_symbol l2cap_disconnect=0x0000ba9d
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0000bced
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0000bd09
--define_symbol l2cap_init=0x0000c0dd
--define_symbol l2cap_le_send_flow_control_credit=0x0000c1d3
--define_symbol l2cap_max_le_mtu=0x0000c4dd
--define_symbol l2cap_register_packet_handler=0x0000c605
--define_symbol l2cap_register_service=0x0000c611
--define_symbol l2cap_request_can_send_now_event=0x0000c721
--define_symbol l2cap_request_connection_parameter_update=0x0000c73b
--define_symbol l2cap_send_echo_request=0x0000cc15
--define_symbol l2cap_unregister_service=0x0000ccd5
--define_symbol le_device_db_add=0x0000cd2d
--define_symbol le_device_db_find=0x0000cdf5
--define_symbol le_device_db_from_key=0x0000ce21
--define_symbol le_device_db_iter_cur=0x0000ce29
--define_symbol le_device_db_iter_cur_key=0x0000ce2d
--define_symbol le_device_db_iter_init=0x0000ce31
--define_symbol le_device_db_iter_next=0x0000ce39
--define_symbol le_device_db_remove_key=0x0000ce5f
--define_symbol ll_aes_encrypt=0x0000ce8d
--define_symbol ll_config=0x0000cf09
--define_symbol ll_free=0x0000cf3f
--define_symbol ll_get_heap_free_size=0x0000cf49
--define_symbol ll_hint_on_ce_len=0x0000cf5d
--define_symbol ll_legacy_adv_set_interval=0x0000cf95
--define_symbol ll_malloc=0x0000cfa5
--define_symbol ll_query_timing_info=0x0000d0dd
--define_symbol ll_register_hci_acl_previewer=0x0000d129
--define_symbol ll_scan_set_fixed_channel=0x0000d18d
--define_symbol ll_set_adv_access_address=0x0000d3a5
--define_symbol ll_set_adv_coded_scheme=0x0000d3b1
--define_symbol ll_set_conn_acl_report_latency=0x0000d3d9
--define_symbol ll_set_conn_coded_scheme=0x0000d405
--define_symbol ll_set_conn_latency=0x0000d42d
--define_symbol ll_set_conn_tx_power=0x0000d459
--define_symbol ll_set_def_antenna=0x0000d49d
--define_symbol ll_set_initiating_coded_scheme=0x0000d4b9
--define_symbol ll_set_max_conn_number=0x0000d4c5
--define_symbol nibble_for_char=0x0001e411
--define_symbol platform_calibrate_rt_clk=0x0001e49f
--define_symbol platform_call_on_stack=0x000040ef
--define_symbol platform_cancel_us_timer=0x0001e4a3
--define_symbol platform_config=0x0001e4b9
--define_symbol platform_controller_run=0x0001e5dd
--define_symbol platform_create_us_timer=0x0001e611
--define_symbol platform_delete_timer=0x0001e625
--define_symbol platform_enable_irq=0x0001e62d
--define_symbol platform_get_gen_os_driver=0x0001e665
--define_symbol platform_get_link_layer_interf=0x0001e671
--define_symbol platform_get_task_handle=0x0001e679
--define_symbol platform_get_timer_counter=0x0001e691
--define_symbol platform_get_us_time=0x0001e695
--define_symbol platform_get_version=0x0001e699
--define_symbol platform_hrng=0x0001e6a1
--define_symbol platform_init_controller=0x0001e6a9
--define_symbol platform_os_idle_resumed_hook=0x0001e6c5
--define_symbol platform_patch_rf_init_data=0x0001e6c9
--define_symbol platform_post_sleep_processing=0x0001e6d5
--define_symbol platform_pre_sleep_processing=0x0001e6db
--define_symbol platform_pre_suppress_ticks_and_sleep_processing=0x0001e6e1
--define_symbol platform_printf=0x0001e6e5
--define_symbol platform_raise_assertion=0x0001e6f9
--define_symbol platform_rand=0x0001e70d
--define_symbol platform_read_info=0x0001e711
--define_symbol platform_read_persistent_reg=0x0001e741
--define_symbol platform_reset=0x0001e751
--define_symbol platform_rt_rc_auto_tune=0x0001e775
--define_symbol platform_rt_rc_auto_tune2=0x0001e77d
--define_symbol platform_rt_rc_tune=0x0001e805
--define_symbol platform_set_abs_timer=0x0001e819
--define_symbol platform_set_evt_callback=0x0001e81d
--define_symbol platform_set_evt_callback_table=0x0001e831
--define_symbol platform_set_irq_callback=0x0001e83d
--define_symbol platform_set_irq_callback_table=0x0001e859
--define_symbol platform_set_rf_clk_source=0x0001e865
--define_symbol platform_set_rf_init_data=0x0001e871
--define_symbol platform_set_rf_power_mapping=0x0001e87d
--define_symbol platform_set_timer=0x0001e889
--define_symbol platform_shutdown=0x0001e88d
--define_symbol platform_switch_app=0x0001e891
--define_symbol platform_trace_raw=0x0001e8bd
--define_symbol platform_write_persistent_reg=0x0001e8d5
--define_symbol printf_hexdump=0x0001e8e5
--define_symbol reverse_128=0x0001ec55
--define_symbol reverse_24=0x0001ec5b
--define_symbol reverse_256=0x0001ec61
--define_symbol reverse_48=0x0001ec67
--define_symbol reverse_56=0x0001ec6d
--define_symbol reverse_64=0x0001ec73
--define_symbol reverse_bd_addr=0x0001ec79
--define_symbol reverse_bytes=0x0001ec7f
--define_symbol sm_add_event_handler=0x0001ef6d
--define_symbol sm_address_resolution_lookup=0x0001f0c5
--define_symbol sm_authenticated=0x0001f46d
--define_symbol sm_authorization_decline=0x0001f47b
--define_symbol sm_authorization_grant=0x0001f49b
--define_symbol sm_authorization_state=0x0001f4bb
--define_symbol sm_bonding_decline=0x0001f4d5
--define_symbol sm_config=0x0001f931
--define_symbol sm_config_conn=0x0001f965
--define_symbol sm_encryption_key_size=0x0001fb1f
--define_symbol sm_just_works_confirm=0x000200a5
--define_symbol sm_le_device_key=0x000203e5
--define_symbol sm_passkey_input=0x0002047b
--define_symbol sm_private_random_address_generation_get=0x00020835
--define_symbol sm_private_random_address_generation_get_mode=0x0002083d
--define_symbol sm_private_random_address_generation_set_mode=0x00020849
--define_symbol sm_private_random_address_generation_set_update_period=0x00020871
--define_symbol sm_register_external_ltk_callback=0x000209ad
--define_symbol sm_register_oob_data_callback=0x000209b9
--define_symbol sm_request_pairing=0x000209c5
--define_symbol sm_send_security_request=0x000214a3
--define_symbol sm_set_accepted_stk_generation_methods=0x000214c9
--define_symbol sm_set_authentication_requirements=0x000214d5
--define_symbol sm_set_encryption_key_size_range=0x000214e5
--define_symbol sscanf_bd_addr=0x00021831
--define_symbol sysSetPublicDeviceAddr=0x00021be5
--define_symbol uuid128_to_str=0x00022391
--define_symbol uuid_add_bluetooth_prefix=0x000223e9
--define_symbol uuid_has_bluetooth_prefix=0x00022409
