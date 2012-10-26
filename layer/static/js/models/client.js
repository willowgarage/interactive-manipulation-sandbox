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
    other_users: DS.attr('string')
  });

});

