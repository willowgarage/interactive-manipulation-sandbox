class AddImageUrlToRobot < ActiveRecord::Migration
  def up
    add_column :robots, :camera_url, :string
  end

  def down
    remove_column :robots, :camera_url
  end
end
