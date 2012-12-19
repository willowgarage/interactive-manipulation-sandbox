/*global module:false*/
module.exports = function(grunt) {

  grunt.initConfig({
    lint: {
      // Files to lint
      all: [
        'grunt.js',
        'js/*.js',
        'js/controllers/*js',
        'js/helpers/*js',
        'js/models/*js',
        'js/views/*js',
      ]
    },
    // JSHint is a JavaScript linter.
    jshint: {
      // Linting options. See http://www.jshint.com/docs/.
      options: {
        // Requires curly braces around blocks
        curly   : true,
        // Requires triple equals in comparisons
        eqeqeq  : true,
        // Wrap functions in () when doing immediate function invokation
        immed   : true,
        // Prevents use of a variable before it was defined
        latedef : true,
        // Capitalize constructor names
        newcap  : true,
        // Prohibits the use of arguments.caller and arguments.callee
        noarg   : true,
        // Allows obj['key'] or obj.key
        sub     : true,
        // Prohibits use of explicitly undefined variables (usually typos).
        undef   : true,
        // Does not allow assignments inside a comparison
        boss    : false,
        // Require === null, not == null
        eqnull  : false,
        // Allows window, document, etc.
        browser : true,
        // Allows console and alert.
        devel   : true,
        // Uses ES5 functions (http://kangax.github.com/es5-compat-table/)
        es5     : true,
        // Ignore strict mode
        strict  : false
      },
      // Global variables and functions ignored by linter.
      globals: {
        define  : true,
        require : true
      }
    }
  });

  grunt.registerTask('dev', 'lint');

};

