class Robot < ActiveRecord::Base
  attr_accessible :description, :locx, :locy, :name
  attr_accessible :image
  has_one :robot_state
  has_attached_file :image, :styles => {:medium=>"300x300>", :thumb => "100x100>"}
end
