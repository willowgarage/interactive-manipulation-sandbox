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
      return (this.get('username') !== "")
    }.property('username')
  });

});

