function v = OP_PROJECT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 86);
  end
  v = vInitialized;
end
