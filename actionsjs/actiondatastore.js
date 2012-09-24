var ActionDataStore = function(options) {
  var actionDataStore = this;
  options = options || {};
  actionDataStore.ros    = options.ros;

  actionDataStore.find = function(query, callback) {
    var closeGripperAction = new Action();
    closeGripperAction.id = 1;
    closeGripperAction.name = 'close_gripper';

    var openGripperAction = new Action();
    openGripperAction.id = 2;
    openGripperAction.name = 'open_gripper';

    var actions = [
      openGripperAction
    , closeGripperAction
    ];
    callback(actions);
  };

  actionDataStore.findOne = function(id, callback) {
    var action = new Action();
    action.id = 1;
    action.name = 'open_gripper';

    callback(action);
  };

  actionDataStore.insert = function(action) {

  };

  actionDataStore.update = function(action) {

  };

  actionDataStore.remove = function(query) {

  };

};
ActionDataStore.prototype.__proto__ = EventEmitter2.prototype;

