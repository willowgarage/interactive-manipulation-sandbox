class RobotState < ActiveRecord::Base
  belongs_to :robots
  attr_accessible :activity, :user
end
