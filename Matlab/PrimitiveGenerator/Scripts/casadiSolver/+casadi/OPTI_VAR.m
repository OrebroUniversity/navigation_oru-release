function v = OPTI_VAR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 161);
  end
  v = vInitialized;
end
