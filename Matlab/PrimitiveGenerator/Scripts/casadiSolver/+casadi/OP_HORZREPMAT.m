function v = OP_HORZREPMAT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 95);
  end
  v = vInitialized;
end
