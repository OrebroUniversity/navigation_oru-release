function v = OP_ADD()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 12);
  end
  v = vInitialized;
end
