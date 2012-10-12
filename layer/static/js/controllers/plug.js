define([
    'ember',
    'app'
], function( Ember, App) {

  App.PlugController = Ember.ObjectController.extend({
    plugIn: function() {
      if( this._toggle()) {
        this.get('content').plugIn();
      }
    },
    unplug: function() {
      if( this._toggle()) {
        this.get('content').unplug();
      }
    },
    _toggle: function() {
      var btns = $('.mybutton');
      if( btns.hasClass('disabled')) {
        return false;
      } else {
        $('.mybutton').addClass('disabled');
        return true;
      }
    }
  });

});

