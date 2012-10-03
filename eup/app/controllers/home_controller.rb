class HomeController < ApplicationController
  def index
    @robots = Robot.find(:all)
  end

end
