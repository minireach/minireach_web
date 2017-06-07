function init() {
  var server_address = '192.168.1.70'
  var ros = new ROSLIB.Ros({
    url : 'ws://' + server_address + ':9090'
  });

  ros.on("connection", function () {
    console.log("conneted!")
  })

  ros.on("error", function (error) {
    console.log("Error!", error)
  })

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  var viewer = new ROS3D.Viewer({
    divID : 'map',
    width : window.innerWidth,
    height : window.innerHeight,
    antialias : true
  });
  viewer.addObject(new ROS3D.Grid());


  new ROS3D.OccupancyGridClient({
    continuous : true,
    ros : new ROSLIB.Ros({
      url : 'ws://' + server_address + ':9090'
    }),
    rootObject : viewer.scene
  });

  /*
  var tfClient = new ROSLIB.TFClient({
    ros : ros,
    angularThres : 0.01,
    transThres : 0.01,
    rate : 10.0,
    fixedFrame : '/map'
  });
  */

  var tfClient2 = new ROSLIB.TFClient({
    ros : ros,
    angularThres : 0.01,
    transThres : 0.01,
    rate : 10.0,
    fixedFrame : '/map'
  });

  /*
  var markerClient = new ROS3D.MarkerClient({
    ros : ros,
    tfClient : tfClient,
    topic : '/position_marker',
    rootObject : viewer.scene
  });
  */

  var urdfClient = new ROS3D.UrdfClient({
    ros : ros,
    tfClient : tfClient2,
    path : 'http://localhost/urdf/',
    rootObject : viewer.scene,
    loader : ROS3D.COLLADA_LOADER_2
  });
}
