define([
  'ember',
  'app',
  'jquery'
], function(Ember, App, $) {

  App.PlugController = Ember.ObjectController.extend({
    plugIn: function() {
      if (this._toggle()) {
        this.get('content').plugIn();
      }
    },
    unplug: function() {
      if (this._toggle()) {
        this.get('content').unplug();
      }
    },

    /* TODO: These two functions (tuck and point) should be deleted when they can
     * be automatically called from unplug */
    tuckArms: function() {
      this.get('content')._tuckArms();
    },

    pointHeadForward: function() {
      this.get('content')._pointHeadForward();
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

