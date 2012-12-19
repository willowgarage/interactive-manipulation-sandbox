define([
  'ember',
  'emberdata',
  'app'
],
function(
  Ember,
  DS,
  App
) {

  App.Client = DS.Model.extend({
    username: DS.attr('string'),
    first_name: DS.attr('string'),
    last_name: DS.attr('string'),
    other_users: DS.attr('string'),
    isLoggedIn: function() {
      var loggedin = (this.get('username') !== 'AnonymousUser');
      return loggedin;
    }.property('username'),
    thereAreOtherUsers: function() {
      return (this.get('other_users') && this.get('other_users').length > 0);
    }.property('other_users'),
    otherUserNames: function() {
      return this.get('other_users') && this.get('other_users').map(function bb(x){ return x.first_name + ' ' + x.last_name; }).join(' and ');
    }.property('other_users')
  });

});

