class CreateRobots < ActiveRecord::Migration
  def change
    create_table :robots do |t|
      t.string :name
      t.string :description
      t.integer :locx
      t.integer :locy

      t.timestamps
    end
  end
end
