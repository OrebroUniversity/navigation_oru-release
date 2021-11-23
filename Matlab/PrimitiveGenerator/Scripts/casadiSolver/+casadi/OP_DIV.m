function v = OP_DIV()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 15);
  end
  v = vInitialized;
end
