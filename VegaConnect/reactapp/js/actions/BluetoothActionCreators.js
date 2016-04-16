var AppDispatcher = require('../dispatcher/AppDispatcher');
var AppConstants = require('../constants/AppConstants');

var ActionTypes = AppConstants.ActionTypes;

module.exports = {

  enable: function() {
    AppDispatcher.dispatch({
      type: ActionTypes.BT_ENABLED
    });
  },

  disable: function() {
    AppDispatcher.dispatch({
      type: ActionTypes.BT_DISABLED
    });
  },

  startScan: function(timeoutMs) {
    AppDispatcher.dispatch({
      type: ActionTypes.BT_SCAN_START,
      timeoutMs: timeoutMs
    });
  },

  stopScan: function() {
    AppDispatcher.dispatch({
      type: ActionTypes.BT_SCAN_CANCEL
    });
  },

  scanFinished: function() {
    AppDispatcher.dispatch({
      type: ActionTypes.BT_SCAN_FINISHED
    });
  },

  deviceFound: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.BT_DEVICE_FOUND,
      address: device.address,
      device: device
    });
  },

  connectDevice: function(address) {
    AppDispatcher.dispatch({
      type: ActionTypes.BT_CONNECT,
      address: address
    });
  },

  disconnectDevice: function(address) {
    AppDispatcher.dispatch({
      type: ActionTypes.BT_DISCONNECT,
      address: address
    });
  },

  deviceConnected: function(address) {
    AppDispatcher.dispatch({
      type: ActionTypes.BT_CONNECTED,
      address: address
    });
  },

  deviceDisconnected: function(address) {
    AppDispatcher.dispatch({
      type: ActionTypes.BT_DISCONNECTED,
      address: address
    });
  }
};
