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

  App.client = Ember.Object.create({
    username: '',
    first_name: '',
    last_name: '',
    other_users: [],
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

