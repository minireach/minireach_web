$(document).ready(function () {


  // ----------OK-BUTTON----------
  $("#ok_button_input").click(function () {
    var pallet_id = $("#pallet_id_input").val();
    var storage_id = $("#storage_id_input").val();


    // Calling a service
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

  // ----------RACK-BUTTON----------
  $("#update_rack_button").click(function(){
    var newRackId = $("#update_rack_input").val();

    var rackActionClient = new ROSLIB.ActionClient({
      ros : ros,
      serverName : '/' + robotName + '/update_rack_pose',
      actionName : 'minireach_tasks/UpdateRackPose'
    });

    var rackGoal = new ROSLIB.Goal({
      actionClient : rackActionClient,
      goalMessage : {
        id : newRackId
      }
    });
    rackGoal.send();
    console.log('Pressed Rack-button');

  });


  // ------------- SAVE-TRUCK-BUTTON --------------
  $("#save_truck_button").click(function(){
    var saveTruckPoseClient = new ROSLIB.Service({
      ros : ros,
      name : '/' + robotName + "/save_truck_pose",
      serviceType : 'std_srvs/Empty'
    })

    var request = new ROSLIB.ServiceRequest({});

    saveTruckPoseClient.callService(
      request,

      function (result) {
        console.log('Result for service call on save truck pose');
      },

      function (error) {
        console.log(Error);
      }
    );
  });

  
  // ----------STORAGE-BUTTON----------
  $("#create_storage_button").click(function(){
    var storage_id = $("#create_storage_input").val();

    var updateStorageClient = new ROSLIB.Service({
      ros : ros,
      name : '/' + robotName + '/update_storage_pose',
      serviceType : 'minireach_rapps/UpdateStorage'
    });


    var request = new ROSLIB.ServiceRequest({
      id : storage_id
    });

    updateStorageClient.callService(
      request,

      function (result) {
        console.log('Result for service call on ' + updateStorageClient.name);
      },

      function (error) {
        console.log(error);
      }
    );
  });

});
