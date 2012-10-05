App = Em.Application.create({});
App.Store = DS.Store.extend({
    adapter:  DS.RESTAdapter.create({namespace:'static/test'}),
    revision: 4
});
App.store = App.Store.create({});

App.Contact  = DS.Model.extend({
  firstName: DS.attr('string'),
  lastName:  DS.attr('string'),
  email:     DS.attr('string'),
  notes:     DS.attr('string'),

  fullName: function() {
    var firstName = this.get('firstName'),
        lastName = this.get('lastName');

    if (!firstName && !lastName) {
      if (this.get('id') === undefined) {
        return '(New Contact)';
      } else {
        return '(No Name)';
      }
    }

    if (firstName === undefined) firstName = '';
    if (lastName === undefined) lastName = '';

    return firstName + ' ' + lastName;
  }.property('firstName', 'lastName'),

  gravatar: function() {
    var email = this.get('email');
    if (!email) email = '';
    return 'http://www.gravatar.com/avatar/' + MD5(email);
  }.property('email')
});


App.ApplicationController = Em.Controller.extend({});
App.ApplicationView = Em.View.extend({
  templateName: 'test/templates/application'
});

App.ContactsController = Em.ArrayController.extend({
  sortProperties: ['lastName', 'firstName']
});
App.ContactsView = Em.View.extend({
  templateName: 'test/templates/contacts'
});

App.ShowContactInListView = Em.View.extend({
  templateName: 'test/templates/show_contact_in_list',
  tagName: 'li',
  classNameBindings: 'isActive:active',

  isActive: function() {
    var id = this.get('content.id'),
        currentPath = App.router.get('currentState.path');

    if (id) {
      return App.get('router.contactController.content.id') === id &&
             currentPath.indexOf('contacts.contact') > -1;
    } else {
      return currentPath.indexOf('contacts.newContact') > -1;
    }
  }.property('App.router.currentState', 'App.router.contactController.content')
});

App.Router = Ember.Router.extend({
  location: 'hash',
  
  root: Em.Route.extend({
    contacts: Em.Route.extend({
      route: '/',
      
      connectOutlets: function(router) {
        router.get('applicationController').connectOutlet('contacts', App.store.findAll(App.Contact));
      },

      index: Em.Route.extend({
        route: '/',

        connectOutlets: function(router) {
          router.get('applicationController').connectOutlet('contacts');
        }
      })
    })
  })
});


Ember.TEMPLATES["test/templates/application"] = Ember.Handlebars.template(function anonymous(Handlebars,depth0,helpers,partials,data) {
helpers = helpers || Ember.Handlebars.helpers;
  var buffer = '', stack1, escapeExpression=this.escapeExpression;


  data.buffer.push("<div class=\"navbar navbar-inverse navbar-fixed-top\">\n  <div class=\"navbar-inner\">\n    <div class=\"container-fluid\">\n      <div class=\"brand\">Ember Contacts</div>\n      <div class=\"btn-group pull-right\">\n        <a class=\"btn\" ");
  stack1 = helpers.action.call(depth0, "showNewContact", {hash:{},contexts:[depth0],data:data});
  data.buffer.push(escapeExpression(stack1) + "><i class=\"icon-plus-sign\"></i> Add Contact</a>\n      </div>\n    </div>\n  </div>\n</div>\n<div class=\"container-fluid\" id=\"main\">\n  <div class=\"row-fluid\">\n    ");
  stack1 = helpers._triageMustache.call(depth0, "outlet", {hash:{},contexts:[depth0],data:data});
  data.buffer.push(escapeExpression(stack1) + "\n  </div>\n</div>\n");
  return buffer;
});
Ember.TEMPLATES["test/templates/contacts"] = Ember.Handlebars.template(function anonymous(Handlebars,depth0,helpers,partials,data) {
helpers = helpers || Ember.Handlebars.helpers;
  var buffer = '', stack1, escapeExpression=this.escapeExpression, self=this;

function program1(depth0,data) {
  
  var buffer = '', stack1;
  data.buffer.push("\n        ");
  stack1 = {};
  stack1['contentBinding'] = "contact";
  stack1 = helpers.view.call(depth0, "App.ShowContactInListView", {hash:stack1,contexts:[depth0],data:data});
  data.buffer.push(escapeExpression(stack1) + "\n      ");
  return buffer;}

  data.buffer.push("<div class=\"span3\">\n  <div class=\"well\">\n    <ul class=\"nav nav-list\">\n      <li class=\"nav-header\">All contacts</li>\n      ");
  stack1 = helpers.each.call(depth0, "contact", "in", "arrangedContent", {hash:{},inverse:self.noop,fn:self.program(1, program1, data),contexts:[depth0,depth0,depth0],data:data});
  if(stack1 || stack1 === 0) { data.buffer.push(stack1); }
  data.buffer.push("\n    </ul>\n  </div>\n</div>\n<div class=\"span9\">\n  ");
  stack1 = helpers._triageMustache.call(depth0, "outlet", {hash:{},contexts:[depth0],data:data});
  data.buffer.push(escapeExpression(stack1) + "\n</div>\n");
  return buffer;
});
Ember.TEMPLATES["test/templates/show_contact_in_list"] = Ember.Handlebars.template(function anonymous(Handlebars,depth0,helpers,partials,data) {
helpers = helpers || Ember.Handlebars.helpers;
  var buffer = '', stack1, escapeExpression=this.escapeExpression;


  data.buffer.push("<a ");
  stack1 = helpers.action.call(depth0, "showContact", "contact", {hash:{},contexts:[depth0,depth0],data:data});
  data.buffer.push(escapeExpression(stack1) + ">");
  stack1 = helpers._triageMustache.call(depth0, "contact.fullName", {hash:{},contexts:[depth0],data:data});
  data.buffer.push(escapeExpression(stack1) + "</a>\n");
  return buffer;
});


App.initialize();
