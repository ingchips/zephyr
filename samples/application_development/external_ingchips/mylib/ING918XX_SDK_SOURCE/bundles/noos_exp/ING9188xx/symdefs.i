--define_symbol att_dispatch_client_can_send_now=0x000057c9
--define_symbol att_dispatch_client_request_can_send_now_event=0x000057cf
--define_symbol att_dispatch_register_client=0x000057d5
--define_symbol att_dispatch_register_server=0x000057e9
--define_symbol att_dispatch_server_can_send_now=0x000057fd
--define_symbol att_dispatch_server_request_can_send_now_event=0x00005803
--define_symbol att_emit_general_event=0x000058b5
--define_symbol att_server_can_send_packet_now=0x00005fe9
--define_symbol att_server_deferred_read_response=0x00005fed
--define_symbol att_server_get_mtu=0x00006005
--define_symbol att_server_indicate=0x0000607d
--define_symbol att_server_init=0x00006101
--define_symbol att_server_notify=0x0000613d
--define_symbol att_server_register_packet_handler=0x00006255
--define_symbol att_server_request_can_send_now_event=0x00006261
--define_symbol att_set_db=0x0000627d
--define_symbol att_set_read_callback=0x00006291
--define_symbol att_set_write_callback=0x0000629d
--define_symbol bd_addr_cmp=0x0000640d
--define_symbol bd_addr_copy=0x00006413
--define_symbol bd_addr_to_str=0x0000641d
--define_symbol big_endian_read_16=0x00006455
--define_symbol big_endian_read_32=0x0000645d
--define_symbol big_endian_store_16=0x00006471
--define_symbol big_endian_store_32=0x0000647d
--define_symbol btstack_config=0x000065b5
--define_symbol btstack_memory_pool_create=0x00006705
--define_symbol btstack_memory_pool_free=0x0000672d
--define_symbol btstack_memory_pool_get=0x0000678d
--define_symbol btstack_push_user_msg=0x000067d5
--define_symbol btstack_push_user_runnable=0x000067e1
--define_symbol btstack_reset=0x000067ed
--define_symbol char_for_nibble=0x00006ae7
--define_symbol gap_add_dev_to_periodic_list=0x000073fd
--define_symbol gap_add_whitelist=0x0000740d
--define_symbol gap_aes_encrypt=0x00007419
--define_symbol gap_clear_white_lists=0x00007451
--define_symbol gap_clr_adv_set=0x0000745d
--define_symbol gap_clr_periodic_adv_list=0x00007469
--define_symbol gap_create_connection_cancel=0x00007475
--define_symbol gap_default_periodic_adv_sync_transfer_param=0x00007481
--define_symbol gap_disconnect=0x00007499
--define_symbol gap_disconnect_all=0x000074c5
--define_symbol gap_ext_create_connection=0x00007505
--define_symbol gap_get_connection_parameter_range=0x000075f9
--define_symbol gap_le_read_channel_map=0x00007635
--define_symbol gap_periodic_adv_create_sync=0x000076a1
--define_symbol gap_periodic_adv_create_sync_cancel=0x000076c5
--define_symbol gap_periodic_adv_set_info_transfer=0x000076d1
--define_symbol gap_periodic_adv_sync_transfer=0x000076e1
--define_symbol gap_periodic_adv_sync_transfer_param=0x000076f1
--define_symbol gap_periodic_adv_term_sync=0x0000770d
--define_symbol gap_read_antenna_info=0x00007795
--define_symbol gap_read_local_tx_power_level=0x000077ad
--define_symbol gap_read_periodic_adv_list_size=0x000077bd
--define_symbol gap_read_phy=0x000077c9
--define_symbol gap_read_remote_tx_power_level=0x000077d5
--define_symbol gap_read_remote_used_features=0x000077e5
--define_symbol gap_read_remote_version=0x000077f1
--define_symbol gap_read_rssi=0x000077fd
--define_symbol gap_remove_whitelist=0x00007809
--define_symbol gap_rmv_adv_set=0x00007885
--define_symbol gap_rmv_dev_from_periodic_list=0x00007891
--define_symbol gap_rx_test_v2=0x000078a1
--define_symbol gap_rx_test_v3=0x000078b1
--define_symbol gap_set_adv_set_random_addr=0x000078fd
--define_symbol gap_set_callback_for_next_hci=0x00007939
--define_symbol gap_set_connection_cte_request_enable=0x00007959
--define_symbol gap_set_connection_cte_response_enable=0x00007975
--define_symbol gap_set_connection_cte_rx_param=0x00007985
--define_symbol gap_set_connection_cte_tx_param=0x000079d9
--define_symbol gap_set_connection_parameter_range=0x00007a25
--define_symbol gap_set_connectionless_cte_tx_enable=0x00007a3d
--define_symbol gap_set_connectionless_cte_tx_param=0x00007a4d
--define_symbol gap_set_connectionless_iq_sampling_enable=0x00007aa9
--define_symbol gap_set_data_length=0x00007b05
--define_symbol gap_set_def_phy=0x00007b1d
--define_symbol gap_set_default_subrate=0x00007b2d
--define_symbol gap_set_ext_adv_data=0x00007b49
--define_symbol gap_set_ext_adv_enable=0x00007b61
--define_symbol gap_set_ext_adv_para=0x00007bd1
--define_symbol gap_set_ext_scan_enable=0x00007ca9
--define_symbol gap_set_ext_scan_para=0x00007cc1
--define_symbol gap_set_ext_scan_response_data=0x00007d61
--define_symbol gap_set_host_channel_classification=0x00007d79
--define_symbol gap_set_path_loss_reporting_enable=0x00007d89
--define_symbol gap_set_path_loss_reporting_param=0x00007d99
--define_symbol gap_set_periodic_adv_data=0x00007db5
--define_symbol gap_set_periodic_adv_enable=0x00007e25
--define_symbol gap_set_periodic_adv_para=0x00007e35
--define_symbol gap_set_periodic_adv_rx_enable=0x00007e4d
--define_symbol gap_set_phy=0x00007e5d
--define_symbol gap_set_random_device_address=0x00007e79
--define_symbol gap_set_tx_power_reporting_enable=0x00007e99
--define_symbol gap_start_ccm=0x00007eb9
--define_symbol gap_subrate_request=0x00007f01
--define_symbol gap_test_end=0x00007f1d
--define_symbol gap_tx_test_v2=0x00007f29
--define_symbol gap_tx_test_v4=0x00007f41
--define_symbol gap_update_connection_parameters=0x00007f65
--define_symbol gap_vendor_tx_continuous_wave=0x00007fa9
--define_symbol gatt_client_cancel_write=0x000084d1
--define_symbol gatt_client_discover_characteristic_descriptors=0x000084f7
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x00008537
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x00008587
--define_symbol gatt_client_discover_characteristics_for_service=0x000085d7
--define_symbol gatt_client_discover_primary_services=0x0000860d
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x0000863f
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x00008683
--define_symbol gatt_client_execute_write=0x000086bf
--define_symbol gatt_client_find_included_services_for_service=0x000086e5
--define_symbol gatt_client_get_mtu=0x00008713
--define_symbol gatt_client_is_ready=0x000087b5
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x000087cb
--define_symbol gatt_client_prepare_write=0x000087ed
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x00008829
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x00008853
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008859
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x00008887
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x0000888d
--define_symbol gatt_client_read_multiple_characteristic_values=0x000088bb
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x000088eb
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x00008919
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x00008965
--define_symbol gatt_client_register_handler=0x000089b1
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x000089bd
--define_symbol gatt_client_signed_write_without_response=0x00008ded
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008eb1
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008eeb
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008f3d
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008f4d
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008f89
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008f99
--define_symbol gatt_client_write_value_of_characteristic=0x00008fd5
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x0000900b
--define_symbol hci_add_event_handler=0x0000a5bd
--define_symbol hci_power_control=0x0000ad6d
--define_symbol hci_register_acl_packet_handler=0x0000af21
--define_symbol kv_commit=0x0000b6bd
--define_symbol kv_get=0x0000b719
--define_symbol kv_init=0x0000b725
--define_symbol kv_init_backend=0x0000b7a5
--define_symbol kv_put=0x0000b7b9
--define_symbol kv_remove=0x0000b7c5
--define_symbol kv_remove_all=0x0000b7f9
--define_symbol kv_value_modified=0x0000b829
--define_symbol kv_value_modified_of_key=0x0000b845
--define_symbol kv_visit=0x0000b851
--define_symbol l2cap_add_event_handler=0x0000b8e1
--define_symbol l2cap_can_send_packet_now=0x0000b8f1
--define_symbol l2cap_create_le_credit_based_connection_request=0x0000baad
--define_symbol l2cap_credit_based_send=0x0000bbf1
--define_symbol l2cap_credit_based_send_continue=0x0000bc1d
--define_symbol l2cap_disconnect=0x0000bc99
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0000bee9
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0000bf05
--define_symbol l2cap_init=0x0000c2d9
--define_symbol l2cap_le_send_flow_control_credit=0x0000c3cf
--define_symbol l2cap_max_le_mtu=0x0000c6d9
--define_symbol l2cap_register_packet_handler=0x0000c801
--define_symbol l2cap_register_service=0x0000c80d
--define_symbol l2cap_request_can_send_now_event=0x0000c91d
--define_symbol l2cap_request_connection_parameter_update=0x0000c937
--define_symbol l2cap_send_echo_request=0x0000ce11
--define_symbol l2cap_unregister_service=0x0000ced1
--define_symbol le_device_db_add=0x0000cf29
--define_symbol le_device_db_find=0x0000d001
--define_symbol le_device_db_from_key=0x0000d02d
--define_symbol le_device_db_iter_cur=0x0000d035
--define_symbol le_device_db_iter_cur_key=0x0000d039
--define_symbol le_device_db_iter_init=0x0000d03d
--define_symbol le_device_db_iter_next=0x0000d045
--define_symbol le_device_db_remove_key=0x0000d06b
--define_symbol ll_ackable_packet_alloc=0x0000d097
--define_symbol ll_ackable_packet_get_status=0x0000d1d1
--define_symbol ll_ackable_packet_run=0x0000d241
--define_symbol ll_ackable_packet_set_tx_data=0x0000d2e5
--define_symbol ll_adjust_conn_peer_tx_power=0x0000d30d
--define_symbol ll_aes_encrypt=0x0000d339
--define_symbol ll_channel_monitor_alloc=0x0000d3b5
--define_symbol ll_channel_monitor_check_each_pdu=0x0000d437
--define_symbol ll_channel_monitor_run=0x0000d49d
--define_symbol ll_config=0x0000d551
--define_symbol ll_conn_abort=0x0000d56d
--define_symbol ll_create_conn=0x0000d5a1
--define_symbol ll_dhkey_generated=0x0000d81d
--define_symbol ll_free=0x0000d851
--define_symbol ll_get_conn_events_info=0x0000d85d
--define_symbol ll_get_conn_info=0x0000d941
--define_symbol ll_get_heap_free_size=0x0000d98d
--define_symbol ll_hint_on_ce_len=0x0000d9a1
--define_symbol ll_install_ecc_engine=0x0000d9d9
--define_symbol ll_legacy_adv_set_interval=0x0000d9e5
--define_symbol ll_lock_frequency=0x0000d9f5
--define_symbol ll_malloc=0x0000da59
--define_symbol ll_p256_key_pair_generated=0x0000da61
--define_symbol ll_raw_packet_alloc=0x0000dc15
--define_symbol ll_raw_packet_free=0x0000dce9
--define_symbol ll_raw_packet_get_bare_rx_data=0x0000dd21
--define_symbol ll_raw_packet_get_iq_samples=0x0000dde7
--define_symbol ll_raw_packet_get_rx_data=0x0000de81
--define_symbol ll_raw_packet_recv=0x0000df21
--define_symbol ll_raw_packet_send=0x0000dfdd
--define_symbol ll_raw_packet_set_bare_data=0x0000e0c5
--define_symbol ll_raw_packet_set_bare_mode=0x0000e103
--define_symbol ll_raw_packet_set_fake_cte_info=0x0000e209
--define_symbol ll_raw_packet_set_param=0x0000e22b
--define_symbol ll_raw_packet_set_rx_cte=0x0000e289
--define_symbol ll_raw_packet_set_tx_cte=0x0000e31f
--define_symbol ll_raw_packet_set_tx_data=0x0000e35d
--define_symbol ll_register_hci_acl_previewer=0x0000e3c1
--define_symbol ll_scan_set_fixed_channel=0x0000e425
--define_symbol ll_scanner_enable_iq_sampling=0x0000e431
--define_symbol ll_set_adv_access_address=0x0000e6e5
--define_symbol ll_set_adv_coded_scheme=0x0000e6f1
--define_symbol ll_set_conn_acl_report_latency=0x0000e721
--define_symbol ll_set_conn_coded_scheme=0x0000e751
--define_symbol ll_set_conn_interval_unit=0x0000e77d
--define_symbol ll_set_conn_latency=0x0000e789
--define_symbol ll_set_conn_tx_power=0x0000e7b9
--define_symbol ll_set_def_antenna=0x0000e7f5
--define_symbol ll_set_initiating_coded_scheme=0x0000e811
--define_symbol ll_set_max_conn_number=0x0000e81d
--define_symbol ll_set_tx_power_range=0x0000e8b1
--define_symbol ll_unlock_frequency=0x0000e8d9
--define_symbol nibble_for_char=0x00021379
--define_symbol platform_calibrate_rt_clk=0x00021415
--define_symbol platform_call_on_stack=0x000040ef
--define_symbol platform_cancel_us_timer=0x00021419
--define_symbol platform_config=0x0002142d
--define_symbol platform_controller_run=0x00021551
--define_symbol platform_create_us_timer=0x00021585
--define_symbol platform_delete_timer=0x00021599
--define_symbol platform_enable_irq=0x000215a1
--define_symbol platform_get_gen_os_driver=0x000215d9
--define_symbol platform_get_link_layer_interf=0x000215e5
--define_symbol platform_get_task_handle=0x000215ed
--define_symbol platform_get_timer_counter=0x00021605
--define_symbol platform_get_us_time=0x00021609
--define_symbol platform_get_version=0x0002160d
--define_symbol platform_hrng=0x00021615
--define_symbol platform_init_controller=0x0002161d
--define_symbol platform_os_idle_resumed_hook=0x00021639
--define_symbol platform_patch_rf_init_data=0x0002163d
--define_symbol platform_post_sleep_processing=0x00021649
--define_symbol platform_pre_sleep_processing=0x0002164f
--define_symbol platform_pre_suppress_ticks_and_sleep_processing=0x00021655
--define_symbol platform_printf=0x00021659
--define_symbol platform_raise_assertion=0x0002166d
--define_symbol platform_rand=0x00021681
--define_symbol platform_read_info=0x00021685
--define_symbol platform_read_persistent_reg=0x000216b5
--define_symbol platform_reset=0x000216c5
--define_symbol platform_rt_rc_auto_tune=0x000216e9
--define_symbol platform_rt_rc_auto_tune2=0x000216f1
--define_symbol platform_rt_rc_tune=0x00021779
--define_symbol platform_set_abs_timer=0x0002178d
--define_symbol platform_set_evt_callback=0x00021791
--define_symbol platform_set_evt_callback_table=0x000217a5
--define_symbol platform_set_irq_callback=0x000217b1
--define_symbol platform_set_irq_callback_table=0x000217cd
--define_symbol platform_set_rf_clk_source=0x000217d9
--define_symbol platform_set_rf_init_data=0x000217e5
--define_symbol platform_set_rf_power_mapping=0x000217f1
--define_symbol platform_set_timer=0x000217fd
--define_symbol platform_shutdown=0x00021801
--define_symbol platform_switch_app=0x00021805
--define_symbol platform_trace_raw=0x00021831
--define_symbol platform_write_persistent_reg=0x00021849
--define_symbol printf_hexdump=0x00021859
--define_symbol reverse_128=0x00021bd1
--define_symbol reverse_24=0x00021bd7
--define_symbol reverse_256=0x00021bdd
--define_symbol reverse_48=0x00021be3
--define_symbol reverse_56=0x00021be9
--define_symbol reverse_64=0x00021bef
--define_symbol reverse_bd_addr=0x00021bf5
--define_symbol reverse_bytes=0x00021bfb
--define_symbol sm_add_event_handler=0x00021f39
--define_symbol sm_address_resolution_lookup=0x0002207d
--define_symbol sm_authenticated=0x000224d5
--define_symbol sm_authorization_decline=0x000224e3
--define_symbol sm_authorization_grant=0x00022503
--define_symbol sm_authorization_state=0x00022523
--define_symbol sm_bonding_decline=0x0002253d
--define_symbol sm_config=0x000229bd
--define_symbol sm_config_conn=0x000229f1
--define_symbol sm_encryption_key_size=0x00022bab
--define_symbol sm_just_works_confirm=0x000238f1
--define_symbol sm_le_device_key=0x00023ce1
--define_symbol sm_numeric_comparison_confirm=0x00023e37
--define_symbol sm_passkey_input=0x00023e75
--define_symbol sm_private_random_address_generation_get=0x0002447d
--define_symbol sm_private_random_address_generation_get_mode=0x00024485
--define_symbol sm_private_random_address_generation_set_mode=0x00024491
--define_symbol sm_private_random_address_generation_set_update_period=0x000244b9
--define_symbol sm_register_external_ltk_callback=0x00024745
--define_symbol sm_register_oob_data_callback=0x00024751
--define_symbol sm_register_sc_oob_data_callback=0x0002475d
--define_symbol sm_request_pairing=0x00024769
--define_symbol sm_sc_generate_oob_data=0x000253a5
--define_symbol sm_send_security_request=0x000254d5
--define_symbol sm_set_accepted_stk_generation_methods=0x000254fd
--define_symbol sm_set_authentication_requirements=0x00025509
--define_symbol sm_set_encryption_key_size_range=0x00025519
--define_symbol sscanf_bd_addr=0x00025925
--define_symbol sysSetPublicDeviceAddr=0x00025cd9
--define_symbol uuid128_to_str=0x000264ad
--define_symbol uuid_add_bluetooth_prefix=0x00026505
--define_symbol uuid_has_bluetooth_prefix=0x00026525
