function [zf, idf, zn, table] = associateknown(x,z,idz,table)
feature = [];
featurelist = [];
idfeature = [];
idlist = [];

for i=1:length(idz)
