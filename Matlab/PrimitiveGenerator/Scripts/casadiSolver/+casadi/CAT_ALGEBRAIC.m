function v = CAT_ALGEBRAIC()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 133);
  end
  v = vInitialized;
end
