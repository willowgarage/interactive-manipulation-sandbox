class ControlController < ApplicationController

  def robot
    @q = params[:q]
    @robot = Robot.find(session[:robot])

    @places = []
    if not @q.nil?
      wildcard = '%' + @q + '%'
      @places = Place.find(:all, :conditions => ["name LIKE ? or tags LIKE ?", wildcard, wildcard])
    end
  end

  def navigate
    @place = Place.find(params[:id])
    @robot = Robot.find(session[:robot])
  end
end
