var keyMirror = require('keymirror');

module.exports = {

  ActionTypes: keyMirror({
    // Bluetooth actions
    BT_ENABLED: null,
    BT_DISABLED: null,
    BT_SCAN_START: null,
    BT_SCAN_CANCEL: null,
    BT_SCAN_FINISHED: null,
    BT_DEVICE_FOUND: null,
    BT_DISCONNECT: null,
    BT_CONNECT: null,
    BT_CONNECTED: null,
    BT_DISCONNECTED: null,

    //UI Interactions
    REFRESH_DEVICE_LIST: null,
    CLICK_DEVICE: null,
    CLICK_DEVICE_SHARE: null,
    CLICK_DEVICE_TAB_OWNER: null,
    CLICK_DEVICE_TAB_POWER: null,
    CLICK_DEVICE_TAB_NETWORK: null,
    CLICK_DEVICE_TAB_ID: null,

    // Get information from device
    REFRESH_DEVICE: null,
    GET_DEVICE_OWNER: null,
    GET_DEVICE_POWER: null,
    GET_DEVICE_NETWORK: null,
    GET_DEVICE_ID: null,

    // Chnage device settings
    SET_DEVICE_OWNER: null,
    SET_DEVICE_POWER: null,
    SET_DEVICE_NETWORK: null
  })

};
