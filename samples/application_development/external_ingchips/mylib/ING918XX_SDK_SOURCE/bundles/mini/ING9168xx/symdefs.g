att_dispatch_client_can_send_now = 0x02004299;
att_dispatch_client_request_can_send_now_event = 0x0200429f;
att_dispatch_register_client = 0x020042a5;
att_dispatch_register_server = 0x020042b9;
att_dispatch_server_can_send_now = 0x020042cd;
att_dispatch_server_request_can_send_now_event = 0x020042d3;
att_emit_general_event = 0x02004385;
att_server_can_send_packet_now = 0x02004ab9;
att_server_deferred_read_response = 0x02004abd;
att_server_get_mtu = 0x02004ad5;
att_server_indicate = 0x02004b41;
att_server_init = 0x02004bc5;
att_server_notify = 0x02004c01;
att_server_register_packet_handler = 0x02004d19;
att_server_request_can_send_now_event = 0x02004d25;
att_set_db = 0x02004d41;
att_set_read_callback = 0x02004d55;
att_set_write_callback = 0x02004d61;
bd_addr_cmp = 0x02004f35;
bd_addr_copy = 0x02004f3b;
bd_addr_to_str = 0x02004f45;
big_endian_read_16 = 0x02004f7d;
big_endian_read_32 = 0x02004f85;
big_endian_store_16 = 0x02004f99;
big_endian_store_32 = 0x02004fa5;
btstack_config = 0x02004fd1;
btstack_memory_pool_create = 0x0200510f;
btstack_memory_pool_free = 0x02005139;
btstack_memory_pool_get = 0x02005199;
btstack_push_user_msg = 0x02005201;
btstack_push_user_runnable = 0x0200520d;
btstack_reset = 0x02005219;
char_for_nibble = 0x0200543f;
eTaskConfirmSleepModeStatus = 0x02005781;
gap_add_dev_to_periodic_list = 0x02005df1;
gap_add_whitelist = 0x02005e01;
gap_aes_encrypt = 0x02005e0d;
gap_clear_white_lists = 0x02005e45;
gap_clr_adv_set = 0x02005e51;
gap_clr_periodic_adv_list = 0x02005e5d;
gap_create_connection_cancel = 0x02005e69;
gap_default_periodic_adv_sync_transfer_param = 0x02005e75;
gap_disconnect = 0x02005e8d;
gap_disconnect_all = 0x02005eb9;
gap_ext_create_connection = 0x02005ef9;
gap_get_connection_parameter_range = 0x02005fd1;
gap_le_read_channel_map = 0x02006009;
gap_periodic_adv_create_sync = 0x02006075;
gap_periodic_adv_create_sync_cancel = 0x02006099;
gap_periodic_adv_set_info_transfer = 0x020060a5;
gap_periodic_adv_sync_transfer = 0x020060b5;
gap_periodic_adv_sync_transfer_param = 0x020060c5;
gap_periodic_adv_term_sync = 0x020060e1;
gap_read_antenna_info = 0x02006169;
gap_read_periodic_adv_list_size = 0x02006175;
gap_read_phy = 0x02006181;
gap_read_remote_used_features = 0x0200618d;
gap_read_remote_version = 0x02006199;
gap_read_rssi = 0x020061a5;
gap_remove_whitelist = 0x020061b1;
gap_rmv_adv_set = 0x0200622d;
gap_rmv_dev_from_periodic_list = 0x02006239;
gap_rx_test_v2 = 0x02006249;
gap_rx_test_v3 = 0x02006259;
gap_set_adv_set_random_addr = 0x020062a5;
gap_set_callback_for_next_hci = 0x020062e1;
gap_set_connection_cte_request_enable = 0x02006301;
gap_set_connection_cte_response_enable = 0x0200631d;
gap_set_connection_cte_rx_param = 0x0200632d;
gap_set_connection_cte_tx_param = 0x02006381;
gap_set_connection_parameter_range = 0x020063cd;
gap_set_connectionless_cte_tx_enable = 0x020063e5;
gap_set_connectionless_cte_tx_param = 0x020063f5;
gap_set_connectionless_iq_sampling_enable = 0x02006451;
gap_set_data_length = 0x020064ad;
gap_set_def_phy = 0x020064c5;
gap_set_ext_adv_data = 0x020064d5;
gap_set_ext_adv_enable = 0x020064ed;
gap_set_ext_adv_para = 0x0200655d;
gap_set_ext_scan_enable = 0x0200662d;
gap_set_ext_scan_para = 0x02006645;
gap_set_ext_scan_response_data = 0x020066e5;
gap_set_host_channel_classification = 0x020066fd;
gap_set_periodic_adv_data = 0x0200670d;
gap_set_periodic_adv_enable = 0x0200677d;
gap_set_periodic_adv_para = 0x0200678d;
gap_set_periodic_adv_rx_enable = 0x020067a5;
gap_set_phy = 0x020067b5;
gap_set_random_device_address = 0x020067d1;
gap_start_ccm = 0x02006801;
gap_test_end = 0x02006849;
gap_tx_test_v2 = 0x02006855;
gap_tx_test_v4 = 0x0200686d;
gap_update_connection_parameters = 0x02006891;
gap_vendor_tx_continuous_wave = 0x020068d1;
gatt_client_cancel_write = 0x02006df9;
gatt_client_discover_characteristic_descriptors = 0x02006e1f;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x02006e5f;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x02006eaf;
gatt_client_discover_characteristics_for_service = 0x02006eff;
gatt_client_discover_primary_services = 0x02006f35;
gatt_client_discover_primary_services_by_uuid128 = 0x02006f67;
gatt_client_discover_primary_services_by_uuid16 = 0x02006fab;
gatt_client_execute_write = 0x02006fe9;
gatt_client_find_included_services_for_service = 0x0200700f;
gatt_client_get_mtu = 0x0200703d;
gatt_client_is_ready = 0x02007101;
gatt_client_listen_for_characteristic_value_updates = 0x02007117;
gatt_client_prepare_write = 0x02007137;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x02007173;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x0200719d;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x020071a3;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x020071d1;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x020071d7;
gatt_client_read_multiple_characteristic_values = 0x02007205;
gatt_client_read_value_of_characteristic_using_value_handle = 0x02007235;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x02007263;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x020072af;
gatt_client_register_handler = 0x020072f9;
gatt_client_reliable_write_long_value_of_characteristic = 0x02007305;
gatt_client_signed_write_without_response = 0x0200770d;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x020077d1;
gatt_client_write_client_characteristic_configuration = 0x0200780b;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x0200785d;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x0200786d;
gatt_client_write_long_value_of_characteristic = 0x020078a9;
gatt_client_write_long_value_of_characteristic_with_offset = 0x020078b9;
gatt_client_write_value_of_characteristic = 0x020078f5;
gatt_client_write_value_of_characteristic_without_response = 0x0200792b;
hci_add_event_handler = 0x02008f05;
hci_power_control = 0x02009699;
hci_register_acl_packet_handler = 0x0200984d;
kv_commit = 0x02009ea9;
kv_get = 0x02009f05;
kv_init = 0x02009f11;
kv_init_backend = 0x02009f8d;
kv_put = 0x02009fa1;
kv_remove = 0x02009fad;
kv_remove_all = 0x02009fe1;
kv_value_modified = 0x0200a011;
kv_value_modified_of_key = 0x0200a02d;
kv_visit = 0x0200a039;
l2cap_add_event_handler = 0x0200a0c9;
l2cap_can_send_packet_now = 0x0200a0d9;
l2cap_create_le_credit_based_connection_request = 0x0200a29d;
l2cap_credit_based_send = 0x0200a3e5;
l2cap_credit_based_send_continue = 0x0200a411;
l2cap_disconnect = 0x0200a48d;
l2cap_get_le_credit_based_connection_credits = 0x0200a679;
l2cap_get_peer_mtu_for_local_cid = 0x0200a695;
l2cap_init = 0x0200aad5;
l2cap_le_send_flow_control_credit = 0x0200abcb;
l2cap_max_le_mtu = 0x0200aed9;
l2cap_register_packet_handler = 0x0200b001;
l2cap_register_service = 0x0200b00d;
l2cap_request_can_send_now_event = 0x0200b11d;
l2cap_request_connection_parameter_update = 0x0200b137;
l2cap_send_echo_request = 0x0200b619;
l2cap_unregister_service = 0x0200b6f9;
le_device_db_add = 0x0200b751;
le_device_db_find = 0x0200b819;
le_device_db_from_key = 0x0200b845;
le_device_db_iter_cur = 0x0200b84d;
le_device_db_iter_cur_key = 0x0200b851;
le_device_db_iter_init = 0x0200b855;
le_device_db_iter_next = 0x0200b85d;
le_device_db_remove_key = 0x0200b883;
ll_adjust_conn_peer_tx_power = 0x0200b8b1;
ll_aes_encrypt = 0x0200b8d9;
ll_config = 0x0200b94d;
ll_free = 0x0200b983;
ll_get_heap_free_size = 0x0200b98d;
ll_hint_on_ce_len = 0x0200b9a1;
ll_legacy_adv_set_interval = 0x0200b9d9;
ll_malloc = 0x0200b9e9;
ll_register_hci_acl_previewer = 0x0200bb01;
ll_scan_set_fixed_channel = 0x0200bb65;
ll_set_adv_access_address = 0x0200bd7d;
ll_set_adv_coded_scheme = 0x0200bd89;
ll_set_conn_acl_report_latency = 0x0200bdb9;
ll_set_conn_coded_scheme = 0x0200bde5;
ll_set_conn_latency = 0x0200be0d;
ll_set_conn_tx_power = 0x0200be39;
ll_set_def_antenna = 0x0200be71;
ll_set_initiating_coded_scheme = 0x0200be91;
ll_set_max_conn_number = 0x0200be9d;
ll_set_tx_power_range = 0x0200bf31;
nibble_for_char = 0x0201c085;
platform_32k_rc_auto_tune = 0x0201c10d;
platform_32k_rc_tune = 0x0201c159;
platform_calibrate_32k = 0x0201c175;
platform_config = 0x0201c179;
platform_delete_timer = 0x0201c2a1;
platform_enable_irq = 0x0201c2a9;
platform_get_current_task = 0x0201c2fd;
platform_get_gen_os_driver = 0x0201c321;
platform_get_heap_status = 0x0201c329;
platform_get_link_layer_interf = 0x0201c341;
platform_get_task_handle = 0x0201c349;
platform_get_timer_counter = 0x0201c369;
platform_get_us_time = 0x0201c36d;
platform_get_version = 0x0201c371;
platform_hrng = 0x0201c379;
platform_install_isr_stack = 0x0201c381;
platform_install_task_stack = 0x0201c38d;
platform_patch_rf_init_data = 0x0201c3c5;
platform_printf = 0x0201c3d1;
platform_raise_assertion = 0x0201c3e5;
platform_rand = 0x0201c3f9;
platform_read_info = 0x0201c3fd;
platform_read_persistent_reg = 0x0201c42d;
platform_reset = 0x0201c439;
platform_set_abs_timer = 0x0201c44d;
platform_set_evt_callback = 0x0201c451;
platform_set_evt_callback_table = 0x0201c465;
platform_set_irq_callback = 0x0201c471;
platform_set_irq_callback_table = 0x0201c48d;
platform_set_rf_clk_source = 0x0201c499;
platform_set_rf_init_data = 0x0201c4a5;
platform_set_rf_power_mapping = 0x0201c4b1;
platform_set_timer = 0x0201c4bd;
platform_shutdown = 0x0201c4c1;
platform_switch_app = 0x0201c4c5;
platform_trace_raw = 0x0201c4dd;
platform_write_persistent_reg = 0x0201c4f5;
printf_hexdump = 0x0201c6ad;
pvPortMalloc = 0x0201d0e1;
pvTaskIncrementMutexHeldCount = 0x0201d1c9;
pvTimerGetTimerID = 0x0201d1e1;
pxPortInitialiseStack = 0x0201d20d;
reverse_128 = 0x0201d3f5;
reverse_24 = 0x0201d3fb;
reverse_256 = 0x0201d401;
reverse_48 = 0x0201d407;
reverse_56 = 0x0201d40d;
reverse_64 = 0x0201d413;
reverse_bd_addr = 0x0201d419;
reverse_bytes = 0x0201d41f;
sm_add_event_handler = 0x0201d725;
sm_address_resolution_lookup = 0x0201d87d;
sm_authenticated = 0x0201dbf9;
sm_authorization_decline = 0x0201dc07;
sm_authorization_grant = 0x0201dc27;
sm_authorization_state = 0x0201dc47;
sm_bonding_decline = 0x0201dc61;
sm_config = 0x0201e0b5;
sm_config_conn = 0x0201e0cd;
sm_encryption_key_size = 0x0201e283;
sm_just_works_confirm = 0x0201e809;
sm_le_device_key = 0x0201eb55;
sm_passkey_input = 0x0201ebeb;
sm_private_random_address_generation_get = 0x0201efa5;
sm_private_random_address_generation_get_mode = 0x0201efad;
sm_private_random_address_generation_set_mode = 0x0201efb9;
sm_private_random_address_generation_set_update_period = 0x0201efe1;
sm_register_external_ltk_callback = 0x0201f11d;
sm_register_oob_data_callback = 0x0201f129;
sm_request_pairing = 0x0201f135;
sm_send_security_request = 0x0201fc0b;
sm_set_accepted_stk_generation_methods = 0x0201fc31;
sm_set_authentication_requirements = 0x0201fc3d;
sm_set_encryption_key_size_range = 0x0201fc49;
sscanf_bd_addr = 0x0201ffa9;
sysSetPublicDeviceAddr = 0x0202005d;
uuid128_to_str = 0x02020805;
uuid_add_bluetooth_prefix = 0x0202085d;
uuid_has_bluetooth_prefix = 0x0202087d;
uxListRemove = 0x02020899;
uxQueueMessagesWaiting = 0x020208c1;
uxQueueMessagesWaitingFromISR = 0x020208e9;
uxQueueSpacesAvailable = 0x02020905;
uxTaskGetStackHighWaterMark = 0x02020931;
uxTaskPriorityGet = 0x02020951;
uxTaskPriorityGetFromISR = 0x0202096d;
vListInitialise = 0x020209d3;
vListInitialiseItem = 0x020209e9;
vListInsert = 0x020209ef;
vListInsertEnd = 0x02020a1f;
vPortEndScheduler = 0x02020a39;
vPortEnterCritical = 0x20000339;
vPortExitCritical = 0x20000373;
vPortFree = 0x02020a65;
vPortSuppressTicksAndSleep = 0x2000039f;
vPortValidateInterruptPriority = 0x02020b09;
vQueueDelete = 0x02020b65;
vQueueWaitForMessageRestricted = 0x02020b91;
vTaskDelay = 0x02020bd5;
vTaskInternalSetTimeOutState = 0x02020c21;
vTaskMissedYield = 0x02020c31;
vTaskPlaceOnEventList = 0x02020c3d;
vTaskPlaceOnEventListRestricted = 0x02020c75;
vTaskPriorityDisinheritAfterTimeout = 0x02020cb5;
vTaskPrioritySet = 0x02020d61;
vTaskResume = 0x02020e29;
vTaskStartScheduler = 0x02020ead;
vTaskStepTick = 0x2000071f;
vTaskSuspend = 0x02020f3d;
vTaskSuspendAll = 0x20000693;
vTaskSwitchContext = 0x02020ff9;
xPortStartScheduler = 0x0202109d;
xQueueAddToSet = 0x020211a1;
xQueueCreateCountingSemaphore = 0x020211c5;
xQueueCreateCountingSemaphoreStatic = 0x02021201;
xQueueCreateMutex = 0x02021245;
xQueueCreateMutexStatic = 0x0202125b;
xQueueCreateSet = 0x02021275;
xQueueGenericCreate = 0x0202127d;
xQueueGenericCreateStatic = 0x020212c9;
xQueueGenericReset = 0x02021331;
xQueueGenericSend = 0x020213bd;
xQueueGenericSendFromISR = 0x02021529;
xQueueGiveFromISR = 0x020215e9;
xQueueGiveMutexRecursive = 0x0202168d;
xQueueIsQueueEmptyFromISR = 0x020216cd;
xQueueIsQueueFullFromISR = 0x020216f1;
xQueuePeek = 0x02021719;
xQueuePeekFromISR = 0x02021841;
xQueueReceive = 0x020218ad;
xQueueReceiveFromISR = 0x020219d9;
xQueueRemoveFromSet = 0x02021a6d;
xQueueSelectFromSet = 0x02021a8f;
xQueueSelectFromSetFromISR = 0x02021aa1;
xQueueSemaphoreTake = 0x02021ab5;
xQueueTakeMutexRecursive = 0x02021c21;
xTaskCheckForTimeOut = 0x02021c65;
xTaskCreate = 0x02021cd5;
xTaskCreateStatic = 0x02021d31;
xTaskGetCurrentTaskHandle = 0x02021da1;
xTaskGetSchedulerState = 0x02021dad;
xTaskGetTickCount = 0x02021dc9;
xTaskGetTickCountFromISR = 0x02021dd5;
xTaskIncrementTick = 0x20000525;
xTaskPriorityDisinherit = 0x02021de5;
xTaskPriorityInherit = 0x02021e79;
xTaskRemoveFromEventList = 0x02021f0d;
xTaskResumeAll = 0x200005df;
xTaskResumeFromISR = 0x02021f8d;
xTimerCreate = 0x02022019;
xTimerCreateStatic = 0x0202204d;
xTimerCreateTimerTask = 0x02022085;
xTimerGenericCommand = 0x020220f1;
xTimerGetExpiryTime = 0x02022161;
xTimerGetTimerDaemonTaskHandle = 0x02022181;
