function v = OP_SUBREF()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 78);
  end
  v = vInitialized;
end
