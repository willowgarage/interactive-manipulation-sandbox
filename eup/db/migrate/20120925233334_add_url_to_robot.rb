class AddUrlToRobot < ActiveRecord::Migration
  def up
    add_column :robots, :url, :string
  end

  def down
    remove_column :robots, :url
  end
end
