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

      action.actionClient = new ActionClient({
        ros        : action.ros,
        actionName : 'executer_actions/ExecuteAction',
        serverName : '/executer/execute'
      });

      var data = {
        name   : action.name,
        inputs : action.inputs
      };
      var serializedData = JSON.stringify(data);
      action.goal = new action.actionClient.Goal({
        action : serializedData
      });

      action.goal.on('result', function(result) {
			  if (result.outputs.length > 0) {
					result.outputs = JSON.parse(result.outputs);
				}
        action.emit('result', result);
      });

      action.goal.on('status', function(status) {
        action.emit('status', status);
      });

      action.goal.on('feedback', function(feedback) {
        action.emit('feedback', feedback);
      });

      action.goal.send();
    },

    action.cancel = function() {
      if (action.goal) {
        action.goal.cancel();
      } else {
        action.cancelAll();
      }
    },

    action.cancelAll = function() {
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

