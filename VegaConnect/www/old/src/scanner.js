var powered = false;
chrome.bluetooth.getAdapterState(function(adapter) {
  powered = adapter.powered;
});

chrome.bluetooth.onAdapterStateChanged.addListener(function(adapter) {
  if (adapter.powered != powered) {
    powered = adapter.powered;
    if (powered) {
      console.log("Adapter radio is on");
      scanForRobots();
    } else {
      console.log("Adapter radio is off");
      chrome.bluetooth.stopDiscovery();
    }
  }
});

chrome.bluetooth.getAdapterState(function(adapter) {
  console.log("Adapter " + adapter.address + ": " + adapter.name);
});

var updateDevice = function(device){
  console.log("Update: ")
  console.log(device);
}

var removeDevice = function(device){
  console.log("Remove: ")
  console.log(device);
}

chrome.bluetooth.onDeviceAdded.addListener(updateDevice);
chrome.bluetooth.onDeviceChanged.addListener(updateDevice);
chrome.bluetooth.onDeviceRemoved.addListener(removeDevice);

var scanForRobots = function(){
  // Now begin the discovery process.
  chrome.bluetooth.startDiscovery(function() {
    // Stop discovery after 30 seconds.
    setTimeout(function() {
      chrome.bluetooth.stopDiscovery(function() {
        console.log("Scan timeout")
      });
    }, 30000);
  });
}
