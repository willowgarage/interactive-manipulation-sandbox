class ChangePlaceLocToFloat < ActiveRecord::Migration
  def up
    change_column :places, :locx, :float
    change_column :places, :locy, :float
    add_column :places, :angle, :float
  end

  def down
    change_column :places, :locx, :integer
    change_column :places, :locy, :integer
    remove_column :places, :angle
  end
end
