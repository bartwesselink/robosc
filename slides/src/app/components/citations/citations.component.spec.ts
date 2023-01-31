import { ComponentFixture, TestBed } from '@angular/core/testing';

import { CitationsComponent } from './citations.component';

describe('CitationsComponent', () => {
  let component: CitationsComponent;
  let fixture: ComponentFixture<CitationsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ CitationsComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(CitationsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
