class AddLocationDetails < ActiveRecord::Migration
  def up
    add_column :places, :width, :integer
    add_column :places, :height, :integer
  end

  def down
    remove_column :places, :width
    remove_column :places, :height
  end
end
