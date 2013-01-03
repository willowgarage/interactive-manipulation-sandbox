define([
  'ember',
  'emberdata',
  'app',
  'jquery'
],
function(
  Ember,
  DS,
  App,
  $
) {

  //  Create "Client" object which will hold information about currently logged-in user
  //  as well as other users connected to the same part of the application
  App.client = Ember.Object.create({
    username: '',
    first_name: '',
    last_name: '',
    other_users: [],
    connection_latency: 0.0,
    isLoggedIn: function() {
      var loggedin = (this.get('username') !== 'AnonymousUser');
      return loggedin;
    }.property('username'),
    thereAreOtherUsers: function() {
      return (this.get('other_users') && this.get('other_users').length > 0);
    }.property('other_users'),
    otherUserNames: function() {
      return this.get('other_users') && this.get('other_users').map(function bb(x){ return x.first_name + ' ' + x.last_name; }).join(' and ');
    }.property('other_users'),
    latency: function(){
      // Return a "integer" with the number of milliseconds for latency.
      return parseInt(this.get('connection_latency').toPrecision(3), 10); // milliseconds
    }.property('connection_latency')
  });

  //  Get currently logged-in information (triggered once per application load)
  $.ajax({
    url: '/client',
    success: function( data) {
      data = eval('(' + data + ')');
      App.client.set('username', data['username']);
      App.client.set('first_name', data['first_name']);
      App.client.set('last_name', data['last_name']);
    }
  });
});
