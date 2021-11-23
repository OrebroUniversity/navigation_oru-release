function v = OP_DETERMINANT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 66);
  end
  v = vInitialized;
end
