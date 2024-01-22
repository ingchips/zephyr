att_dispatch_client_can_send_now = 0x00005799;
att_dispatch_client_request_can_send_now_event = 0x0000579f;
att_dispatch_register_client = 0x000057a5;
att_dispatch_register_server = 0x000057b9;
att_dispatch_server_can_send_now = 0x000057cd;
att_dispatch_server_request_can_send_now_event = 0x000057d3;
att_emit_general_event = 0x00005885;
att_server_can_send_packet_now = 0x00005fb9;
att_server_deferred_read_response = 0x00005fbd;
att_server_get_mtu = 0x00005fd5;
att_server_indicate = 0x0000604d;
att_server_init = 0x000060d1;
att_server_notify = 0x0000610d;
att_server_register_packet_handler = 0x00006225;
att_server_request_can_send_now_event = 0x00006231;
att_set_db = 0x0000624d;
att_set_read_callback = 0x00006261;
att_set_write_callback = 0x0000626d;
bd_addr_cmp = 0x000063dd;
bd_addr_copy = 0x000063e3;
bd_addr_to_str = 0x000063ed;
big_endian_read_16 = 0x00006425;
big_endian_read_32 = 0x0000642d;
big_endian_store_16 = 0x00006441;
big_endian_store_32 = 0x0000644d;
btstack_config = 0x000065a1;
btstack_memory_pool_create = 0x000066df;
btstack_memory_pool_free = 0x00006709;
btstack_memory_pool_get = 0x00006769;
btstack_push_user_msg = 0x000067d1;
btstack_push_user_runnable = 0x000067dd;
btstack_reset = 0x000067e9;
char_for_nibble = 0x00006ac5;
eTaskConfirmSleepModeStatus = 0x00006da9;
gap_add_dev_to_periodic_list = 0x000073a5;
gap_add_whitelist = 0x000073b5;
gap_aes_encrypt = 0x000073c1;
gap_clear_white_lists = 0x000073f9;
gap_clr_adv_set = 0x00007405;
gap_clr_periodic_adv_list = 0x00007411;
gap_create_connection_cancel = 0x0000741d;
gap_disconnect = 0x00007429;
gap_disconnect_all = 0x00007455;
gap_ext_create_connection = 0x00007495;
gap_get_connection_parameter_range = 0x0000756d;
gap_le_read_channel_map = 0x000075a9;
gap_periodic_adv_create_sync = 0x00007615;
gap_periodic_adv_create_sync_cancel = 0x00007639;
gap_periodic_adv_term_sync = 0x00007645;
gap_read_periodic_adv_list_size = 0x000076cd;
gap_read_phy = 0x000076d9;
gap_read_remote_used_features = 0x000076e5;
gap_read_remote_version = 0x000076f1;
gap_read_rssi = 0x000076fd;
gap_remove_whitelist = 0x00007709;
gap_rmv_adv_set = 0x00007785;
gap_rmv_dev_from_periodic_list = 0x00007791;
gap_rx_test_v2 = 0x000077a1;
gap_set_adv_set_random_addr = 0x000077d9;
gap_set_callback_for_next_hci = 0x00007815;
gap_set_connection_parameter_range = 0x00007835;
gap_set_data_length = 0x0000784d;
gap_set_def_phy = 0x00007865;
gap_set_ext_adv_data = 0x00007875;
gap_set_ext_adv_enable = 0x0000788d;
gap_set_ext_adv_para = 0x000078fd;
gap_set_ext_scan_enable = 0x000079d5;
gap_set_ext_scan_para = 0x000079ed;
gap_set_ext_scan_response_data = 0x00007a8d;
gap_set_host_channel_classification = 0x00007aa5;
gap_set_periodic_adv_data = 0x00007ab5;
gap_set_periodic_adv_enable = 0x00007b25;
gap_set_periodic_adv_para = 0x00007b35;
gap_set_phy = 0x00007b4d;
gap_set_random_device_address = 0x00007b69;
gap_start_ccm = 0x00007b99;
gap_test_end = 0x00007be1;
gap_tx_test_v2 = 0x00007bed;
gap_tx_test_v4 = 0x00007c05;
gap_update_connection_parameters = 0x00007c29;
gap_vendor_tx_continuous_wave = 0x00007c6d;
gatt_client_cancel_write = 0x00008195;
gatt_client_discover_characteristic_descriptors = 0x000081bb;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x000081fb;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x0000824b;
gatt_client_discover_characteristics_for_service = 0x0000829b;
gatt_client_discover_primary_services = 0x000082d1;
gatt_client_discover_primary_services_by_uuid128 = 0x00008303;
gatt_client_discover_primary_services_by_uuid16 = 0x00008347;
gatt_client_execute_write = 0x00008383;
gatt_client_find_included_services_for_service = 0x000083a9;
gatt_client_get_mtu = 0x000083d7;
gatt_client_is_ready = 0x00008479;
gatt_client_listen_for_characteristic_value_updates = 0x0000848f;
gatt_client_prepare_write = 0x000084b1;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x000084ed;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x00008517;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x0000851d;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x0000854b;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x00008551;
gatt_client_read_multiple_characteristic_values = 0x0000857f;
gatt_client_read_value_of_characteristic_using_value_handle = 0x000085af;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x000085dd;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x00008629;
gatt_client_register_handler = 0x00008675;
gatt_client_reliable_write_long_value_of_characteristic = 0x00008681;
gatt_client_signed_write_without_response = 0x00008ab1;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x00008b75;
gatt_client_write_client_characteristic_configuration = 0x00008baf;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x00008c01;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008c11;
gatt_client_write_long_value_of_characteristic = 0x00008c4d;
gatt_client_write_long_value_of_characteristic_with_offset = 0x00008c5d;
gatt_client_write_value_of_characteristic = 0x00008c99;
gatt_client_write_value_of_characteristic_without_response = 0x00008ccf;
hci_add_event_handler = 0x0000a1f5;
hci_power_control = 0x0000a991;
hci_register_acl_packet_handler = 0x0000ab45;
kv_commit = 0x0000b2b9;
kv_get = 0x0000b315;
kv_init = 0x0000b321;
kv_init_backend = 0x0000b3a1;
kv_put = 0x0000b3b5;
kv_remove = 0x0000b3c1;
kv_remove_all = 0x0000b3f5;
kv_value_modified = 0x0000b425;
kv_value_modified_of_key = 0x0000b441;
kv_visit = 0x0000b44d;
l2cap_add_event_handler = 0x0000b4dd;
l2cap_can_send_packet_now = 0x0000b4ed;
l2cap_create_le_credit_based_connection_request = 0x0000b6a9;
l2cap_credit_based_send = 0x0000b7ed;
l2cap_credit_based_send_continue = 0x0000b819;
l2cap_disconnect = 0x0000b895;
l2cap_get_le_credit_based_connection_credits = 0x0000bae5;
l2cap_get_peer_mtu_for_local_cid = 0x0000bb01;
l2cap_init = 0x0000bed5;
l2cap_le_send_flow_control_credit = 0x0000bfcb;
l2cap_max_le_mtu = 0x0000c2d5;
l2cap_register_packet_handler = 0x0000c3fd;
l2cap_register_service = 0x0000c409;
l2cap_request_can_send_now_event = 0x0000c519;
l2cap_request_connection_parameter_update = 0x0000c533;
l2cap_send_echo_request = 0x0000ca0d;
l2cap_unregister_service = 0x0000cacd;
le_device_db_add = 0x0000cb25;
le_device_db_find = 0x0000cbfd;
le_device_db_from_key = 0x0000cc29;
le_device_db_iter_cur = 0x0000cc31;
le_device_db_iter_cur_key = 0x0000cc35;
le_device_db_iter_init = 0x0000cc39;
le_device_db_iter_next = 0x0000cc41;
le_device_db_remove_key = 0x0000cc67;
ll_aes_encrypt = 0x0000cc95;
ll_config = 0x0000cd11;
ll_free = 0x0000cd47;
ll_get_heap_free_size = 0x0000cd51;
ll_hint_on_ce_len = 0x0000cd65;
ll_legacy_adv_set_interval = 0x0000cd9d;
ll_malloc = 0x0000cdad;
ll_query_timing_info = 0x0000cee5;
ll_register_hci_acl_previewer = 0x0000cf31;
ll_scan_set_fixed_channel = 0x0000cf95;
ll_set_adv_access_address = 0x0000d1ad;
ll_set_adv_coded_scheme = 0x0000d1b9;
ll_set_conn_acl_report_latency = 0x0000d1e9;
ll_set_conn_coded_scheme = 0x0000d219;
ll_set_conn_latency = 0x0000d245;
ll_set_conn_tx_power = 0x0000d275;
ll_set_def_antenna = 0x0000d2bd;
ll_set_initiating_coded_scheme = 0x0000d2d9;
ll_set_max_conn_number = 0x0000d2e5;
nibble_for_char = 0x0001d49d;
platform_calibrate_rt_clk = 0x0001d549;
platform_call_on_stack = 0x00004183;
platform_cancel_us_timer = 0x0001d54d;
platform_config = 0x0001d561;
platform_create_us_timer = 0x0001d685;
platform_delete_timer = 0x0001d699;
platform_enable_irq = 0x0001d6a1;
platform_get_current_task = 0x0001d6d9;
platform_get_gen_os_driver = 0x0001d6fd;
platform_get_heap_status = 0x0001d705;
platform_get_link_layer_interf = 0x0001d71d;
platform_get_task_handle = 0x0001d725;
platform_get_timer_counter = 0x0001d745;
platform_get_us_time = 0x0001d749;
platform_get_version = 0x0001d74d;
platform_hrng = 0x0001d755;
platform_install_isr_stack = 0x0001d75d;
platform_install_task_stack = 0x0001d769;
platform_patch_rf_init_data = 0x0001d7a1;
platform_printf = 0x0001d7ad;
platform_raise_assertion = 0x0001d7c1;
platform_rand = 0x0001d7d5;
platform_read_info = 0x0001d7d9;
platform_read_persistent_reg = 0x0001d809;
platform_reset = 0x0001d819;
platform_rt_rc_auto_tune = 0x0001d83d;
platform_rt_rc_auto_tune2 = 0x0001d845;
platform_rt_rc_tune = 0x0001d8cd;
platform_set_abs_timer = 0x0001d8e1;
platform_set_evt_callback = 0x0001d8e5;
platform_set_evt_callback_table = 0x0001d8f9;
platform_set_irq_callback = 0x0001d905;
platform_set_irq_callback_table = 0x0001d921;
platform_set_rf_clk_source = 0x0001d92d;
platform_set_rf_init_data = 0x0001d939;
platform_set_rf_power_mapping = 0x0001d945;
platform_set_timer = 0x0001d951;
platform_shutdown = 0x0001d955;
platform_switch_app = 0x0001d959;
platform_trace_raw = 0x0001d985;
platform_write_persistent_reg = 0x0001d99d;
printf_hexdump = 0x0001db51;
pvPortMalloc = 0x0001e645;
pvTaskIncrementMutexHeldCount = 0x0001e72d;
pvTimerGetTimerID = 0x0001e745;
pxPortInitialiseStack = 0x0001e771;
reverse_128 = 0x0001e919;
reverse_24 = 0x0001e91f;
reverse_256 = 0x0001e925;
reverse_48 = 0x0001e92b;
reverse_56 = 0x0001e931;
reverse_64 = 0x0001e937;
reverse_bd_addr = 0x0001e93d;
reverse_bytes = 0x0001e943;
sm_add_event_handler = 0x0001eae1;
sm_address_resolution_lookup = 0x0001ec39;
sm_authenticated = 0x0001efe1;
sm_authorization_decline = 0x0001efef;
sm_authorization_grant = 0x0001f00f;
sm_authorization_state = 0x0001f02f;
sm_bonding_decline = 0x0001f049;
sm_config = 0x0001f4a5;
sm_config_conn = 0x0001f4d9;
sm_encryption_key_size = 0x0001f693;
sm_just_works_confirm = 0x0001fc19;
sm_le_device_key = 0x0001ff59;
sm_passkey_input = 0x0001ffef;
sm_private_random_address_generation_get = 0x000203a9;
sm_private_random_address_generation_get_mode = 0x000203b1;
sm_private_random_address_generation_set_mode = 0x000203bd;
sm_private_random_address_generation_set_update_period = 0x000203e5;
sm_register_external_ltk_callback = 0x00020521;
sm_register_oob_data_callback = 0x0002052d;
sm_request_pairing = 0x00020539;
sm_send_security_request = 0x00021017;
sm_set_accepted_stk_generation_methods = 0x0002103d;
sm_set_authentication_requirements = 0x00021049;
sm_set_encryption_key_size_range = 0x00021059;
sscanf_bd_addr = 0x000213a5;
sysSetPublicDeviceAddr = 0x00021759;
uuid128_to_str = 0x00021d6d;
uuid_add_bluetooth_prefix = 0x00021dc5;
uuid_has_bluetooth_prefix = 0x00021de5;
uxListRemove = 0x00021e01;
uxQueueMessagesWaiting = 0x00021e29;
uxQueueMessagesWaitingFromISR = 0x00021e51;
uxQueueSpacesAvailable = 0x00021e6d;
uxTaskGetStackHighWaterMark = 0x00021e99;
uxTaskPriorityGet = 0x00021eb9;
uxTaskPriorityGetFromISR = 0x00021ed5;
vListInitialise = 0x00021f8f;
vListInitialiseItem = 0x00021fa5;
vListInsert = 0x00021fab;
vListInsertEnd = 0x00021fdb;
vPortEndScheduler = 0x00021ff5;
vPortEnterCritical = 0x00022021;
vPortExitCritical = 0x00022065;
vPortFree = 0x00022099;
vPortSuppressTicksAndSleep = 0x0002212d;
vPortValidateInterruptPriority = 0x00022255;
vQueueDelete = 0x000222b1;
vQueueWaitForMessageRestricted = 0x000222dd;
vTaskDelay = 0x00022325;
vTaskInternalSetTimeOutState = 0x00022371;
vTaskMissedYield = 0x00022381;
vTaskPlaceOnEventList = 0x0002238d;
vTaskPlaceOnEventListRestricted = 0x000223c5;
vTaskPriorityDisinheritAfterTimeout = 0x00022405;
vTaskPrioritySet = 0x000224b1;
vTaskResume = 0x00022579;
vTaskStartScheduler = 0x000225fd;
vTaskStepTick = 0x0002268d;
vTaskSuspend = 0x000226bd;
vTaskSuspendAll = 0x00022779;
vTaskSwitchContext = 0x00022789;
xPortStartScheduler = 0x00022839;
xQueueAddToSet = 0x00022901;
xQueueCreateCountingSemaphore = 0x00022925;
xQueueCreateCountingSemaphoreStatic = 0x00022961;
xQueueCreateMutex = 0x000229a5;
xQueueCreateMutexStatic = 0x000229bb;
xQueueCreateSet = 0x000229d5;
xQueueGenericCreate = 0x000229dd;
xQueueGenericCreateStatic = 0x00022a29;
xQueueGenericReset = 0x00022a91;
xQueueGenericSend = 0x00022b1d;
xQueueGenericSendFromISR = 0x00022c89;
xQueueGiveFromISR = 0x00022d49;
xQueueGiveMutexRecursive = 0x00022ded;
xQueueIsQueueEmptyFromISR = 0x00022e2d;
xQueueIsQueueFullFromISR = 0x00022e51;
xQueuePeek = 0x00022e79;
xQueuePeekFromISR = 0x00022fa1;
xQueueReceive = 0x0002300d;
xQueueReceiveFromISR = 0x00023139;
xQueueRemoveFromSet = 0x000231cd;
xQueueSelectFromSet = 0x000231ef;
xQueueSelectFromSetFromISR = 0x00023201;
xQueueSemaphoreTake = 0x00023215;
xQueueTakeMutexRecursive = 0x00023381;
xTaskCheckForTimeOut = 0x000233c5;
xTaskCreate = 0x00023435;
xTaskCreateStatic = 0x00023491;
xTaskGetCurrentTaskHandle = 0x00023501;
xTaskGetSchedulerState = 0x0002350d;
xTaskGetTickCount = 0x00023529;
xTaskGetTickCountFromISR = 0x00023535;
xTaskIncrementTick = 0x00023545;
xTaskPriorityDisinherit = 0x00023611;
xTaskPriorityInherit = 0x000236a5;
xTaskRemoveFromEventList = 0x00023739;
xTaskResumeAll = 0x000237b9;
xTaskResumeFromISR = 0x00023881;
xTimerCreate = 0x0002390d;
xTimerCreateStatic = 0x00023941;
xTimerCreateTimerTask = 0x00023979;
xTimerGenericCommand = 0x000239e5;
xTimerGetExpiryTime = 0x00023a55;
xTimerGetTimerDaemonTaskHandle = 0x00023a75;
