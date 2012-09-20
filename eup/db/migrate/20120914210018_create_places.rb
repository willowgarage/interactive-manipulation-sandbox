class CreatePlaces < ActiveRecord::Migration
  def change
    create_table :places do |t|
      t.string :name
      t.string :tags
      t.integer :locx
      t.integer :locy

      t.timestamps
    end
  end
end
