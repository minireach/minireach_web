$(document).ready(function () {

  $("#ok_button_input").click(function () {
    var pallet_id = $("#pallet_id_input").val();
    var storage_id = $("#storage_id_input").val();
    //alert("Pallet ID: " + pallet_id + "\nStorage ID: " + storage_id);

    // Calling a service
    // -----------------

    var sendMissionClient = new ROSLIB.Service({
      ros : ros,
      name : '/' + robotName + '/send_mission',
      serviceType : 'minireach_executive/StartMission'
    });

    var request = new ROSLIB.ServiceRequest({
      pallet_id : pallet_id,
      storage_id : storage_id
    });

    sendMissionClient.callService(
      request,

      function (result) {
        console.log('Result for service call on ' + sendMissionClient.name);
      },

      function (error) {
        console.log(error);
      }
    );

  });
});


function startHandlePallet() {
  //Handle pallet instance
  var handlePalletClient = new ROSLIB.Service({
    ros : ros,
    name : '/' + robotName + '/start_rapp',
    serviceType : 'rocon_app_manager_msgs/StartRapp'
  });

  var request = new ROSLIB.ServiceRequest({
    name : 'minireach_rapps/handle_pallets_mapping'
  });

  handlePalletClient.callService(
    request,

    function (result) {
      console.log('Result for service call on ' + handlePalletClient.name);
    },

    function (error) {
      console.log(error);
    }
  );
  //rosservice call /espeon/start_rapp "name: 'minireach_rapps/handle_pallets_mapping'
}
