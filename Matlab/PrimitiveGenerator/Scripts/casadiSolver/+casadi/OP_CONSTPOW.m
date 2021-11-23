function v = OP_CONSTPOW()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 20);
  end
  v = vInitialized;
end
