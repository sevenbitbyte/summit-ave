var AppDispatcher = require('../dispatcher/AppDispatcher');
var AppConstants = require('../constants/AppConstants');

var ActionTypes = AppConstants.ActionTypes;

module.exports = {

  refresh: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.REFRESH_DEVICE,
      device: device
    });
  },

  getDeviceOwner: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.GET_DEVICE_OWNER,
      device: device
    });
  },

  getDevicePower: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.GET_DEVICE_POWER,
      device: device
    });
  },

  getDeviceNetwork: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.GET_DEVICE_NETWORK,
      device: device
    });
  },

  getDeviceId: function(device) {
    AppDispatcher.dispatch({
      type: ActionTypes.GET_DEVICE_ID,
      device: device
    });
  },


  setDeviceOwner: function(device, params) {
    AppDispatcher.dispatch({
      type: ActionTypes.SET_DEVICE_OWNER,
      device: device,
      params: params
    });
  },

  setDevicePower: function(device, params) {
    AppDispatcher.dispatch({
      type: ActionTypes.SET_DEVICE_POWER,
      device: device,
      params: params
    });
  },

  setDeviceNetwork: function(device, params) {
    AppDispatcher.dispatch({
      type: ActionTypes.SET_DEVICE_NETWORK,
      device: device,
      params: params
    });
  }
};
