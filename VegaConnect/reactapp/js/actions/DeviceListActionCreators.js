var AppDispatcher = require('../dispatcher/AppDispatcher');
var AppConstants = require('../constants/AppConstants');

var ActionTypes = AppConstants.ActionTypes;

module.exports = {

  refresh: function() {
    AppDispatcher.dispatch({
      type: ActionTypes.REFRESH_DEVICE_LIST
    });
  },

  clickDevice: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.CLICK_DEVICE,
      device: device
    });
  },

  clickShareDevice: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.CLICK_DEVICE_SHARE,
      device: device
    });
  },

  clickDeviceOwner: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.CLICK_DEVICE_TAB_OWNER,
      device: device
    });
  },

  clickDevicePower: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.CLICK_DEVICE_TAB_POWER,
      device: device
    });
  },

  clickDeviceNetwork: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.CLICK_DEVICE_TAB_NETWORK,
      device: device
    });
  },

  clickDeviceId: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.CLICK_DEVICE_TAB_ID,
      device: device
    });
  }

};
