(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define([ 'eventemitter2', 'actionclient'], factory);
  } else {
    root.Action = factory(root.EventEmitter2, root.ActionClient);
  }
}(this, function (EventEmitter2, ActionClient) {

  var Action = function(options) {
    var action = this;
    options = options || {};
    action.ros    = options.ros;
    action.name   = options.name;
    action.inputs = options.inputs || {};

    action.execute = function() {

      var actionClient = new ActionClient({
        ros        : action.ros,
        actionName : 'executer_actions/ExecuteAction',
        serverName : '/executer/execute'
      });

      var data = {
        name   : action.name,
        inputs : action.inputs
      };
      var serializedData = JSON.stringify(data);
      var goal = new actionClient.Goal({
        action : serializedData
      });

      goal.on('result', function(result) {
        action.emit('result', result);
      });

      goal.on('status', function(status) {
        action.emit('status', status);
      });

      goal.on('feedback', function(feedback) {
        action.emit('feedback', feedback);
      });

      goal.send();
    },

    action.cancel = function() {

      var actionClient = new ActionClient({
        ros         : action.ros,
        actionName : 'executer_actions/ExecuteAction',
        serverName : '/executer/execute'
      });

      actionClient.cancel();
    }
  };
  Action.prototype.__proto__ = EventEmitter2.prototype;

  return Action;
}));

