function v = OP_CEIL()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 38);
  end
  v = vInitialized;
end
