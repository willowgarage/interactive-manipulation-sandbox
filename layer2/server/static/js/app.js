define([
  'ember',
  'emberdata',
  'jquery',
  'socketio'
],
function(
  Ember,
  DS,
  $,
  io
) {
  var App = Ember.Application.create({
    autoinit: false,

    ready: function() {
      //  Create a master socket connection to server
      this.socket = io.connect('/client', {

        // Maximum number of milliseconds between reconnect attempts.
        'reconnection limit': 3000,

        // Attempt to reconnect for roughly 5 minutes.
        'max reconnection attempts': 100
      });

      // Disconnect-reconnect routine.
      this.socket.on('disconnect', function(){
        $.blockUI({ message: '<h2>Connection to server lost...</h2><img src="/static/img/spinnerLarge.gif"/><h2>...trying to reconnect.</h2>'});
      });
      this.socket.on('reconnecting', function(){
        // Event fired inmediatly before an attempt to reach the server.
        console.log('...trying to reconnect...');
      });
      this.socket.on('reconnect', function(){
        $.unblockUI();
      });

      this.socket.on('context_others',function(other_users){
        console.log("Notified about other users", other_users);
        App.client.set('other_users', other_users);
      });
    },

    setClientContext: function( clientContext) {
      App.client.set('context', clientContext);
      this.socket.emit('context_new', clientContext);
    }
  });
  window.TheApp = App;

  /* Overriding behavior in RESTAdapter to handle responses of the type
      [{ object },{ object }, ...]
    as returned by default in django-rest-framework
    instead of currently required:
      { robots: [{ object },{ object }, ...]}

    NOTE: This can be greatly reduced in code size by generalizing the
    code that repeats between functions. I'm leaving it out for now
    to focus on the basics, but I should contribute this soon
  */
  DS.DjangoRESTAdapter = DS.RESTAdapter.extend({
    find: function( store, type, id) {
      var root = this.rootForType(type);
      this.ajax( this.buildURL(root, id), 'GET', {
        success: function( djson) {
          var json = {}; json[root] = djson;
          this.sideload(store, type, json, root);
          store.load(type, json[root]);
        }
      });
    },
    findMany: function( store, type, ids) {
      ids = this.get('serializer').serializeIds(ids);
      var root = this.rootForType(type), plural = this.pluralize(root);
      this.ajax( this.buildURL(root), 'GET', {
        data: { ids: ids },
        success: function( djson) {
          var json = {}; json[plural] = djson;
          this.sideload(store, type, json, plural);
          store.loadMany(type, json[plural]);
        }
      });
    },
    findAll: function( store, type) {
      var root = this.rootForType(type), plural = this.pluralize(root);
      this.ajax( this.buildURL(root), 'GET', {
        success: function( djson) {
          var json = {}; json[plural] = djson;
          this.sideload(store, type, json, plural);
          store.loadMany(type, json[plural]);
        }
      });
    },
    findQuery: function( store, type, query, recordArray) {
      var root = this.rootForType(type), plural = this.pluralize(root);
      this.ajax( this.buildURL(root), 'GET', {
        data: query,
        success: function(djson) {
          var json = {}; json[plural] = djson;
          this.sideload(store, type, json, plural);
          recordArray.load(json[plural]);
        }
      });
    }
  });

  App.store = DS.Store.create({
    revision: 6,
    adapter: DS.DjangoRESTAdapter.create({ namespace: 'world/api' })
  });

  return App;
});
