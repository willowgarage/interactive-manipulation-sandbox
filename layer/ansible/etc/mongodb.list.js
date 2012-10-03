var result = {};
for(coll in config) {
    result[coll] = db[coll].find().map(function(x){ return x[config[coll]]; });
}
printjson(result);
