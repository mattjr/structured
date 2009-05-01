
for i=0:1
  for j=0:3
    t=sprintf('stuff-%d-%d.txt',i,j);
    fp=    fopen(t,'r');
    pt=fscanf(fp,'%f %f %f\n',[3 inf]);
    c=sprintf('+%d',mod(i+j,6));
    plot3(pt(1,:),pt(2,:),pt(3,:),c)
    hold on
  end
end
