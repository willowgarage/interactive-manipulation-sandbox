class HomeController < ApplicationController
  def index
    @robots = Robot.find(:all)
  end

  def search
    @q = params[:q]
    @robot = Robot.find(session[:robot])

    @places = []
    if not @q.nil?
      wildcard = '%' + @q + '%'
      @places = Place.find(:all, :conditions => ["name LIKE ? or tags LIKE ?", wildcard, wildcard])
    end
  end
end
